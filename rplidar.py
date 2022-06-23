'''Simple and lightweight module for working with RPLidar rangefinder scanners.

Usage example:

>>> from rplidar import RPLidar
>>> lidar = RPLidar('/dev/ttyUSB0')
>>>
>>> info = lidar.get_info()
>>> print(info)
>>>
>>> health = lidar.get_health()
>>> print(health)
>>>
>>> for i, scan in enumerate(lidar.iter_scans()):
...  print('%d: Got %d measures' % (i, len(scan)))
...  if i > 10:
...   break
...
>>> lidar.stop()
>>> lidar.stop_motor()
>>> lidar.disconnect()

For additional information please refer to the RPLidar class documentation.
'''
from encodings import utf_8
import logging
import sys
import time
import codecs
from turtle import distance
import serial
import struct
from collections import namedtuple

SYNC_S_BYTE = b'\xA5'
SYNC_R_BYTE = b'\x5A'


CMD_SCAN               = b'\x20'
CMD_FORCE_SCAN         = b'\x21'
CMD_STOP               = b'\x25'
CMD_RESET              = b'\x40'
CMD_GET_INFO           = b'\x50'
CMD_GET_HEALTH         = b'\x52'
CMD_GET_SAMPLERATE     = b'\x59'     #added in fw 1.17
CMD_EXPRESS_SCAN       = b'\x82'     #added in fw 1.17
CMD_HQ_SCAN            = b'\x83'     #added in fw 1.24
CMD_GET_LIDAR_CONF     = b'\x84'     #added in fw 1.24
CMD_SET_LIDAR_CONF     = b'\x85'     #added in fw 1.24
#add for A2 to set RPLIDAR motor pwm when using accessory board
CMD_SET_MOTOR_PWM      = b'\xF0'
CMD_GET_ACC_BOARD_FLAG = b'\xFF'


ANS_TYPE_DEVINFO                    = 4
ANS_TYPE_DEVHEALTH                  = 6
ANS_TYPE_MEASUREMENT                = 0x81
# Added in FW ver 1.17
ANS_TYPE_MEASUREMENT_CAPSULED       = 0x82
ANS_TYPE_MEASUREMENT_HQ             = 0x83
ANS_TYPE_SAMPLE_RATE                = 0x15
#added in FW ver 1.23alpha
ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA = 0x84
#added in FW ver 1.24
ANS_TYPE_GET_LIDAR_CONF             = 0x20
ANS_TYPE_SET_LIDAR_CONF             = 0x21
ANS_TYPE_MEASUREMENT_DENSE_CAPSULED = 0x85
ANS_TYPE_ACC_BOARD_FLAG             = 0xFF

DESCRIPTOR_LEN          = 7
GET_INFO_LEN            = 20
GET_HEALTH_LEN          = 3
GET_ACC_BOARD_LEN       = 4
GET_SAMPLERATE_LEN      = 4
MEASUREMENT_LEN         = 5
MEASUREMENT_EXPRESS_LEN = 84
MEASUREMENT_CAPSULED_ULTRA_LEN = 132

CONF_SCAN_MODE_COUNT         = b'\x70\x00\x00\x00'
CONF_SCAN_MODE_US_PER_SAMPLE = b'\x71\x00\x00\x00'
CONF_SCAN_MODE_MAX_DISTANCE  = b'\x74\x00\x00\x00'
CONF_SCAN_MODE_ANS_TYPE      = b'\x75\x00\x00\x00'
CONF_SCAN_MODE_TYPICAL       = b'\x7C\x00\x00\x00'
CONF_SCAN_MODE_NAME          = b'\x7F\x00\x00\x00'

_SCAN_TYPE = {
    'normal':    {'byte': CMD_SCAN,         'response': ANS_TYPE_MEASUREMENT,                'size': MEASUREMENT_LEN},
    'force':     {'byte': CMD_FORCE_SCAN,   'response': ANS_TYPE_MEASUREMENT,                'size': MEASUREMENT_LEN},
    'express':   {'byte': CMD_EXPRESS_SCAN, 'response': ANS_TYPE_MEASUREMENT_CAPSULED,       'size': MEASUREMENT_EXPRESS_LEN},
    'extended':  {'byte': CMD_EXPRESS_SCAN, 'response': ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA, 'size': MEASUREMENT_CAPSULED_ULTRA_LEN},
    'dense':     {'byte': CMD_EXPRESS_SCAN, 'response': ANS_TYPE_MEASUREMENT_DENSE_CAPSULED, 'size': MEASUREMENT_EXPRESS_LEN},
}


# Constants & Command to start A2 motor
MAX_MOTOR_PWM = 1023
DEFAULT_MOTOR_PWM = 660


_HEALTH_STATUSES = {
    0: 'Good',
    1: 'Warning',
    2: 'Error',
}


class RPLidarException(Exception):
    '''Basic exception class for RPLidar'''


def _b2i(byte):
    '''Converts byte to integer (for Python 2 compatability)'''
    return byte if int(sys.version[0]) == 3 else ord(byte)


def _showhex(signal):
    '''Converts string bytes to hex representation (useful for debugging)'''
    return [format(_b2i(b), '#02x') for b in signal]

def _anglediff(currAngle, prevAngle):
    diffAngle = currAngle - prevAngle
    if prevAngle > currAngle:
        diffAngle += 360
    return diffAngle


def _process_scan(raw):
    '''Processes input raw data and returns measurement data'''
    new_scan = bool(_b2i(raw[0]) & 0b1)
    inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
    quality = _b2i(raw[0]) >> 2
    if new_scan == inversed_new_scan:
        raise RPLidarException('New scan flags mismatch')
    check_bit = _b2i(raw[1]) & 0b1
    if check_bit != 1:
        raise RPLidarException('Check bit not equal to 1')
    angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
    distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
    return new_scan, quality, angle, distance


def _process_express_scan(data, new_angle, trame):
    new_scan = (new_angle < data.start_angle) & (trame == 1)
    angle = (data.start_angle + ( (new_angle - data.start_angle) % 360 )/32*trame - data.angle[trame-1]) % 360
    distance = data.distance[trame-1]
    return new_scan, None, angle, distance

def _process_dense_scan(data, new_angle, cabin):
    new_scan = (new_angle < data.start_angle) & (cabin == 1)
    angle = data.start_angle + _anglediff(new_angle, data.start_angle)/40*cabin
    distance = data.distance[cabin-1]
    return new_scan, None, angle, distance



class RPLidar(object):
    '''Class for communicating with RPLidar rangefinder scanners'''

    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        '''Initilize RPLidar object for communicating with the sensor.

        Parameters
        ----------
        port : str
            Serial port name to which sensor is connected
        baudrate : int, optional
            Baudrate for serial connection (the default is 115200)
        timeout : float, optional
            Serial port connection timeout in seconds (the default is 1)
        logger : logging.Logger instance, optional
            Logger instance, if none is provided new instance is created
        '''
        self._serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._motor_speed = DEFAULT_MOTOR_PWM
        self.scanning = [False, 0, 'normal']
        self.express_trame = 32
        self.express_data = False
        self.dense_cabin = 40
        self.dense_data = False
        self.motor_running = None
        if logger is None:
            logger = logging.getLogger('rplidar')
        self.logger = logger
        self.connect()

    def connect(self):
        '''Connects to the serial port with the name `self.port`. If it was
        connected to another serial port disconnects from it first.'''
        if self._serial is not None:
            self.disconnect()
        try:
            self._serial = serial.Serial(
                self.port, self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout)
        except serial.SerialException as err:
            raise RPLidarException('Failed to connect to the sensor '
                                   'due to: %s' % err)

    def disconnect(self):
        '''Disconnects from the serial port'''
        if self._serial is None:
            return
        self._serial.close()

    def _set_pwm(self, pwm):
        payload = struct.pack("<H", pwm)
        self._send_payload_cmd(CMD_SET_MOTOR_PWM, payload)

    @property
    def motor_speed(self):
        return self._motor_speed

    @motor_speed.setter
    def motor_speed(self, pwm):
        assert(0 <= pwm <= MAX_MOTOR_PWM)
        self._motor_speed = pwm
        if self.motor_running:
            self._set_pwm(self._motor_speed)

    def start_motor(self):
        '''Starts sensor motor'''
        self.logger.info('Starting motor')
        # For A1
        self._serial.setDTR(False)

        # For A2
        self._set_pwm(self._motor_speed)
        self.motor_running = True

    def stop_motor(self):
        '''Stops sensor motor'''
        self.logger.info('Stoping motor')
        # For A2
#        self._set_pwm(0)
#        time.sleep(.001)
        # For A1
        self._serial.setDTR(True)
        time.sleep(.001)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self.motor_running = False

    def _send_payload_cmd(self, cmd, payload):
        '''Sends `cmd` command with `payload` to the sensor'''
        size = struct.pack('B', len(payload))
        req = SYNC_S_BYTE + cmd + size + payload
        checksum = 0
        for v in struct.unpack('B'*len(req), req):
            checksum ^= v
        req += struct.pack('B', checksum)
        self._serial.write(req)
        self.logger.debug('Command sent: %s' % _showhex(req))

    def _send_cmd(self, cmd):
        '''Sends `cmd` command to the sensor'''
        req = SYNC_S_BYTE + cmd
        self._serial.write(req)
        self.logger.debug('Command sent: %s' % _showhex(req))

    def _read_descriptor(self):
        '''Reads descriptor packet'''
        descriptor = self._serial.read(DESCRIPTOR_LEN)
        self.logger.debug('Received descriptor: %s', _showhex(descriptor))
        if len(descriptor) != DESCRIPTOR_LEN:
            raise RPLidarException('Descriptor length mismatch')
        elif not descriptor.startswith(SYNC_S_BYTE + SYNC_R_BYTE):
            raise RPLidarException('Incorrect descriptor starting bytes')
        is_single = _b2i(descriptor[-2]) == 0
        return _b2i(descriptor[2]), is_single, _b2i(descriptor[-1])

    def _read_response(self, dsize):
        '''Reads response packet with length of `dsize` bytes'''
        self.logger.debug('Trying to read response: %d bytes', dsize)
        while self._serial.inWaiting() < dsize:
            time.sleep(0.001)
        data = self._serial.read(dsize)
        self.logger.debug('Received data: %s', _showhex(data))
        return data

    def get_info(self):
        '''Get device information

        Returns
        -------
        dict
            Dictionary with the sensor information
        '''
        if self._serial.inWaiting() > 0:
            return ('Data in buffer, you can\'t have info ! '
                    'Run clean_input() to emptied the buffer.')
        self._send_cmd(CMD_GET_INFO)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != GET_INFO_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != ANS_TYPE_DEVINFO:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        serialnumber = codecs.encode(raw[4:], 'hex').upper()
        serialnumber = codecs.decode(serialnumber, 'ascii')
        data = {
            'model': _b2i(raw[0]),
            'firmware': (_b2i(raw[2]), _b2i(raw[1])),
            'hardware': _b2i(raw[3]),
            'serialnumber': serialnumber,
        }
        return data


    def get_health(self):
        '''Get device health state. When the core system detects some
        potential risk that may cause hardware failure in the future,
        the returned status value will be 'Warning'. But sensor can still work
        as normal. When sensor is in the Protection Stop state, the returned
        status value will be 'Error'. In case of warning or error statuses
        non-zero error code will be returned.

        Returns
        -------
        status : str
            'Good', 'Warning' or 'Error' statuses
        error_code : int
            The related error code that caused a warning/error.
        '''
        if self._serial.inWaiting() > 0:
            return ('Data in buffer, you can\'t have info ! '
                    'Run clean_input() to emptied the buffer.')
        self.logger.info('Asking for health')
        self._send_cmd(CMD_GET_HEALTH)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != GET_HEALTH_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != ANS_TYPE_DEVHEALTH:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        status = _HEALTH_STATUSES[_b2i(raw[0])]
        error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
        return status, error_code

    def get_MotorCtrlSupport(self):
        '''Get device supporting Motor control
        returns false for A1,  true for A2, not known yet for S1

        Returns
        -------
        bool
            Flag supporting motor control
        '''
        if self._serial.inWaiting() > 0:
            return ('Data in buffer, you can\'t have info ! '
                    'Run clean_input() to emptied the buffer.')
        self.logger.info('Asking for support motor control')
        self._send_payload_cmd(CMD_GET_ACC_BOARD_FLAG, b'\x00\x00\x00\x00')
        try:
            dsize, is_single, dtype = self._read_descriptor()
        except:
            return False
        if dsize != GET_ACC_BOARD_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != ANS_TYPE_ACC_BOARD_FLAG:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        support = (raw[0] & 0x1 == 1)
        return support

    def get_SampleRate(self):
        '''Get single sampling time

        Returns
        -------
            std_rate : int
                time for take one sample in STANDARD mode in uS
            explr_rate : int
                time for take one sample in EXPRESS mode in uS
        '''
        if self._serial.inWaiting() > 0:
            return ('Data in buffer, you can\'t have info ! '
                    'Run clean_input() to emptied the buffer.')
        self.logger.info('Asking for sample rate timings')
        self._send_cmd(CMD_GET_SAMPLERATE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != GET_SAMPLERATE_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != ANS_TYPE_SAMPLE_RATE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        std_rate = (_b2i(raw[1]) << 8) + _b2i(raw[0])
        expr_rate = (_b2i(raw[3]) << 8) + _b2i(raw[2])
        return std_rate, expr_rate

    def get_LidarConf(self, conf, add_payload=None):
        '''Get LIDAR configuration
        Parameters
        ----------
        type : int
            configuration entry
        payload : str
            additional payload

        Returns
        raw : str
            raw data response
        -------
        '''
        if self._serial.inWaiting() > 0:
            return ('Data in buffer, you can\'t have info ! '
                    'Run clean_input() to emptied the buffer.')
        self.logger.info('Asking for lidar configuration entry')
        if add_payload == None:
            payload = conf
        else:
            payload = conf + add_payload
        self._send_payload_cmd(CMD_GET_LIDAR_CONF, payload)
        dsize, is_single, dtype = self._read_descriptor()
        if not is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != ANS_TYPE_GET_LIDAR_CONF:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        return raw

    def get_ScanModeCount(self):
        ''' Get the amount of scan modes supported by LIDAR

        RPLIDAR returns the amount of scan modes supported when receives this command.

        RPLIDAR supports scan mode ids from 0 to (scan_mode_count – 1).
        For instance, device returning 2 according to this query means that the device
        support 2 work modes, whose ids are 0, 1. The host system may use the work mode
        id and other configuration type to get specific characters of the work mode.

        Returns
        -------
        count : int
            amount of scan modes supported
        '''
        raw = self.get_LidarConf(CONF_SCAN_MODE_COUNT)
        count = (_b2i(raw[5]) << 8) + _b2i(raw[4])
        return count

    def get_TypicalScanMode(self):
        ''' Get the typical scan mode id of LIDAR
        no payload in request
        must return uint 16

        Returns
        -------
        mode : int
            id of typical scan mode
        '''
        raw = self.get_LidarConf(CONF_SCAN_MODE_TYPICAL)
        id = (_b2i(raw[5]) << 8) + _b2i(raw[4])
        return id

    def get_ScanModeName(self, mode):
        ''' Get the name of scan mode, whose id is specified by the payload of request.
        The return value is a string of a user-friendly name for this scan mode.

        Parameters
        ----------
        mode : int
            id of scan mode

        Returns
        -------
        name : str
            user-friendly name for this scan mode.
        '''
        pl = struct.pack("<H", mode)
        raw = self.get_LidarConf(CONF_SCAN_MODE_NAME, pl);
        # return str(raw[4:-1], "utf-8")
        return str(raw[4:], "ascii")

    def get_ScanModeAnsType(self, mode):
        ''' Get the answer command type of the scan mode, whose id is specified by the payload of request.
        The return value is 8bit unsigned int, denotes the answer command type.
        Typical return answer types:
        0x81 – For standard mode, returns data in rplidar_resp_measurement_node_t
        0x82 – For express mode, returns data in capsuled format
        0x83 – For boost, stability and sensitivity mode, returns data in ultra capsuled format

        Parameters
        ----------
        mode : int
            id of scan mode

        Returns
        -------
        ans_type : int
        '''
        pl = struct.pack("<H", mode)
        raw = self.get_LidarConf(CONF_SCAN_MODE_ANS_TYPE, pl);
        return _b2i(raw[4])

    def get_MaxDistance(self, mode):
        ''' Get max measurement distance of the scan mode, whose id is specified by the payload of request.

        Parameters
        ----------
        mode : int
            id of scan mode

        Returns
        -------
        distance : float
            distance in meters
        '''
        pl = struct.pack("<H", mode)
        raw = self.get_LidarConf(CONF_SCAN_MODE_MAX_DISTANCE, pl)
        # distance = _b2i(raw[4])/256.0 + _b2i(raw[5]) + (_b2i(raw[6])<<8) + (_b2i(raw[7])<<16)
        # return distance
        distance = struct.unpack('I', raw[4:])
        return distance[0]/256.0

    def get_LidarSampleDuration(self, mode):
        '''Get sample duration of the scan mode, whose id is specified by the payload of request.

        Parameters
        ----------
        mode : int
            id of scan mode

        Returns
        -------
        duration: float
            sample duration in microseseconds
        '''
        pl = struct.pack("<H", mode)
        raw = self.get_LidarConf(CONF_SCAN_MODE_US_PER_SAMPLE, pl)
        # duration = _b2i(raw[4])/256.0 + _b2i(raw[5]) + (_b2i(raw[6])<<8) + (_b2i(raw[7])<<16)
        # return duration
        duration = struct.unpack('I', raw[4:])
        return duration[0]/256.0



    def clean_input(self):
        '''Clean input buffer by reading all available data'''
        if self.scanning[0]:
            return 'Cleanning not allowed during scanning process active !'
        self._serial.flushInput()
        self.express_trame = 32
        self.express_data = False

    def stop(self):
        '''Stops scanning process, disables laser diode and the measurement
        system, moves sensor to the idle state.'''
        self.logger.info('Stopping scanning')
        self._send_cmd(CMD_STOP)
        time.sleep(.1)
        self.scanning[0] = False
        self.clean_input()

    def start(self, scan_type='normal', express_mode=0):
        '''Start the scanning process

        Parameters
        ----------
        scan : normal, force, express, extended or dense.
        express_mode : id of express scan mode, 0 for legacy express scan mode
        '''
        if self.scanning[0]:
            return 'Scanning already running !'
        '''Start the scanning process, enable laser diode and the
        measurement system'''
        status, error_code = self.get_health()
        self.logger.debug('Health status: %s [%d]', status, error_code)
        if status == _HEALTH_STATUSES[2]:
            self.logger.warning('Trying to reset sensor due to the error. '
                                'Error code: %d', error_code)
            self.reset()
            status, error_code = self.get_health()
            if status == _HEALTH_STATUSES[2]:
                raise RPLidarException('RPLidar hardware failure. '
                                       'Error code: %d' % error_code)
        elif status == _HEALTH_STATUSES[1]:
            self.logger.warning('Warning sensor status detected! '
                                'Error code: %d', error_code)

        cmd = _SCAN_TYPE[scan_type]['byte']
        self.logger.info('starting scan process in %s mode' % scan_type)

        if (scan_type != 'standard') or (scan_type != 'force'):
            self._send_payload_cmd(cmd, b'\x00\x00\x00\x00\x00')
        else:
            self._send_cmd(cmd)

        dsize, is_single, dtype = self._read_descriptor()
        if dsize != _SCAN_TYPE[scan_type]['size']:
            raise RPLidarException('Wrong get_info reply length')
        if is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != _SCAN_TYPE[scan_type]['response']:
            raise RPLidarException('Wrong response data type, get %d expected %d', dtype, _SCAN_TYPE[scan_type]['response'])
        self.scanning = [True, dsize, scan_type]



    def reset(self):
        '''Resets sensor core, reverting it to a similar state as it has
        just been powered up.'''
        self.logger.info('Reseting the sensor')
        self._send_cmd(CMD_RESET)
        time.sleep(2)
        self.clean_input()

    def iter_measures(self, scan_type='normal', max_buf_meas=3000):
        '''Iterate over measures. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increasing lag.

        Parameters
        ----------
        max_buf_meas : int or False if you want unlimited buffer
            Maximum number of bytes to be stored inside the buffer. Once
            numbe exceeds this limit buffer will be emptied out.

        Yields
        ------
        new_scan : bool
            True if measures belongs to a new scan
        quality : int
            Reflected laser pulse strength
        angle : float
            The measure heading angle in degree unit [0, 360)
        distance : float
            Measured object distance related to the sensor's rotation center.
            In millimeter unit. Set to 0 when measure is invalid.
        '''
        self.start_motor()
        if not self.scanning[0]:
            self.start(scan_type)
        while True:
            dsize = self.scanning[1]
            if max_buf_meas:
                data_in_buf = self._serial.inWaiting()
                if data_in_buf > max_buf_meas:
                    self.logger.warning(
                        'Too many bytes in the input buffer: %d/%d. '
                        'Cleaning buffer...',
                        data_in_buf, max_buf_meas)
                    self.stop()
                    self.start(self.scanning[2])

            if self.scanning[2] == 'normal':
                raw = self._read_response(dsize)
                yield _process_scan(raw)
            if self.scanning[2] == 'express':
                if self.express_trame == 32:
                    self.express_trame = 0
                    if not self.express_data:
                        self.logger.debug('reading first time bytes')
                        self.express_data = ExpressPacket.from_string(
                                            self._read_response(dsize))

                    self.express_old_data = self.express_data
                    self.logger.debug('set old_data with start_angle %f',
                                      self.express_old_data.start_angle)
                    self.express_data = ExpressPacket.from_string(
                                        self._read_response(dsize))
                    self.logger.debug('set new_data with start_angle %f',
                                      self.express_data.start_angle)

                self.express_trame += 1
                self.logger.debug('process scan of frame %d with angle : '
                                  '%f and angle new : %f', self.express_trame,
                                  self.express_old_data.start_angle,
                                  self.express_data.start_angle)
                yield _process_express_scan(self.express_old_data,
                                            self.express_data.start_angle,
                                            self.express_trame)
            if self.scanning[2] == 'dense':
                if self.dense_cabin == 40:
                    self.dense_cabin = 0
                    if not self.dense_data:
                        self.logger.debug('reading first time bytes')
                        self.dense_data = DensePacket.from_string(self._read_response(dsize))

                    self.dense_old_data = self.dense_data
                    self.logger.debug('set dense old_data with start_angle %f', self.dense_old_data.start_angle)
                    self.dense_data = DensePacket.from_string(self._read_response(dsize))
                    self.logger.debug('set dense new_data with start_angle %f', self.dense_data.start_angle)

                self.dense_cabin += 1
                self.logger.debug('process scan of frame %d with angle : %f and angle new : %f', self.dense_cabin, self.dense_old_data.start_angle, self.dense_data.start_angle)
                yield _process_dense_scan(self.dense_old_data, self.dense_data.start_angle, self.dense_cabin)


    def iter_scans(self, scan_type='normal', max_buf_meas=3000, min_len=5):
        '''Iterate over scans. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increasing lag.

        Parameters
        ----------
        max_buf_meas : int
            Maximum number of measures to be stored inside the buffer. Once
            numbe exceeds this limit buffer will be emptied out.
        min_len : int
            Minimum number of measures in the scan for it to be yelded.

        Yields
        ------
        scan : list
            List of the measures. Each measurment is tuple with following
            format: (quality, angle, distance). For values description please
            refer to `iter_measures` method's documentation.
        '''
        scan_list = []
        iterator = self.iter_measures(scan_type, max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan_list) > min_len:
                    yield scan_list
                scan_list = []
            if distance > 0:
                scan_list.append((quality, angle, distance))


class ExpressPacket(namedtuple('express_packet',
                               'distance angle new_scan start_angle')):
    sync1 = 0xa
    sync2 = 0x5
    sign = {0: 1, 1: -1}

    @classmethod
    def from_string(cls, data):
        packet = bytearray(data)

        if (packet[0] >> 4) != cls.sync1 or (packet[1] >> 4) != cls.sync2:
            raise ValueError('try to parse corrupted data ({})'.format(packet))

        checksum = 0
        for b in packet[2:]:
            checksum ^= b
        if checksum != (packet[0] & 0b00001111) + ((
                        packet[1] & 0b00001111) << 4):
            raise ValueError('Invalid checksum ({})'.format(packet))

        new_scan = packet[3] >> 7
        start_angle = (packet[2] + ((packet[3] & 0b01111111) << 8)) / 64

        d = a = ()
        for i in range(0,80,5):
            d += ((packet[i+4] >> 2) + (packet[i+5] << 6),)
            a += (((packet[i+8] & 0b00001111) + ((
                    packet[i+4] & 0b00000001) << 4))/8*cls.sign[(
                     packet[i+4] & 0b00000010) >> 1],)
            d += ((packet[i+6] >> 2) + (packet[i+7] << 6),)
            a += (((packet[i+8] >> 4) + (
                (packet[i+6] & 0b00000001) << 4))/8*cls.sign[(
                    packet[i+6] & 0b00000010) >> 1],)
        return cls(d, a, new_scan, start_angle)


class DensePacket(namedtuple('dense_packet',
                               'distance new_scan start_angle')):
    sync1 = 0xa
    sync2 = 0x5
    sign = {0: 1, 1: -1}

    @classmethod
    def from_string(cls, data):
        packet = bytearray(data)

        if (packet[0] >> 4) != cls.sync1 or (packet[1] >> 4) != cls.sync2:
            raise ValueError('try to parse corrupted data ({})'.format(packet))

        checksum = 0
        for b in packet[2:]:
            checksum ^= b
        if checksum != (packet[0] & 0b00001111) + ((
                        packet[1] & 0b00001111) << 4):
            raise ValueError('Invalid checksum ({})'.format(packet))

        new_scan = packet[3] >> 7
        start_angle = (packet[2] + ((packet[3] & 0b01111111) << 8)) / 64

        d = ()
        for i in range(0,80,2):
            d += (packet[i+4] + (packet[i+5]<<8),)
        return cls(d, new_scan, start_angle)
