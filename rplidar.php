<?php

include 'PhpSerial.php';

define("SYNC_S_BYTE", 0xA5);
define("SYNC_R_BYTE", 0x5A);

define("CMD_SCAN",           0x20);
define("CMD_FORCE_SCAN",     0x21);

define("CMD_STOP",           0x25);
define("CMD_RESET",          0x40);
define("CMD_GET_INFO",       0x50);
define("CMD_GET_HEALTH",     0x52);
define("CMD_GET_SAMPLERATE", 0x59);       //added in fw 1.17

define("CMD_EXPRESS_SCAN",   0x82);       //added in fw 1.17
define("CMD_HQ_SCAN",        0x83);       //added in fw 1.24
define("CMD_GET_LIDAR_CONF", 0x84);       //added in fw 1.24
define("CMD_SET_LIDAR_CONF", 0x85);       //added in fw 1.24

//add for A2 to set RPLIDAR motor pwm when using accessory board
define("CMD_SET_MOTOR_PWM",      0xF0);
define("CMD_GET_ACC_BOARD_FLAG", 0xFF);

define("MAX_MOTOR_PWM",    1023);
define("DEFAULT_MOTOR_PWM", 660);

define("SEND_MODE_SINGLE", 0);
define("SEND_MODE_MULTI",  1);
define("SEND_MODE_RSRVD1", 2);
define("SEND_MODE_RSRVD2", 3);

// Response
define("ANS_TYPE_DEVINFO",                    0x04);
define("ANS_TYPE_DEVHEALTH",                  0x06);
define("ANS_TYPE_MEASUREMENT",                0x81);
// Added in FW ver 1.17
define("ANS_TYPE_MEASUREMENT_CAPSULED",       0x82);
define("ANS_TYPE_MEASUREMENT_HQ",             0x83);
// Added in FW ver 1.17
define("ANS_TYPE_SAMPLE_RATE",                0x15);
//added in FW ver 1.23alpha
define("ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA", 0x84);
//added in FW ver 1.24
define("ANS_TYPE_GET_LIDAR_CONF",             0x20);
define("ANS_TYPE_SET_LIDAR_CONF",             0x21);
define("ANS_TYPE_MEASUREMENT_DENSE_CAPSULED", 0x85);
define("ANS_TYPE_ACC_BOARD_FLAG",             0xFF);

define("DESCRIPTOR_LEN",     7);
define("GET_INFO_LEN",      20);
define("GET_HEALTH_LEN",     3);
define("GET_ACC_BOARD_LEN",  4);
define("GET_SAMPLERATE_LEN", 4);
define("MEASUREMENT_LEN",    5);
define("MEASUREMENT_EXPRESS_LEN", 84);


define("SCAN_TYPE_STANDARD",    0);
define("SCAN_TYPE_EXPRESS",     1);
// define("SCAN_TYPE_BOOST",       2);
// define("SCAN_TYPE_SENSITIVITY", 3);
// define("SCAN_TYPE_STABILITY",   4);
// define("SCAN_TYPE_DENSE_BOOST", 5);

// define("CONF_SCAN_COMMAND_STD",         0);
// define("CONF_SCAN_COMMAND_EXPRESS",     1);
// define("CONF_SCAN_COMMAND_HQ",          2);
// define("CONF_SCAN_COMMAND_BOOST",       3);
// define("CONF_SCAN_COMMAND_STABILITY",   4);
// define("CONF_SCAN_COMMAND_SENSITIVITY", 5);

// define("CONF_ANGLE_RANGE",                0x00000000);
// define("CONF_DESIRED_ROT_FREQ",           0x00000001);
// define("CONF_SCAN_COMMAND_BITMAP",        0x00000002);
// define("CONF_MIN_ROT_FREQ",               0x00000004);
// define("CONF_MAX_ROT_FREQ",               0x00000005);
// define("CONF_MAX_DISTANCE",               0x00000060);
        
define("CONF_SCAN_MODE_COUNT",            0x00000070);
define("CONF_SCAN_MODE_US_PER_SAMPLE",    0x00000071);
define("CONF_SCAN_MODE_MAX_DISTANCE",     0x00000074);
define("CONF_SCAN_MODE_ANS_TYPE",         0x00000075);
define("CONF_SCAN_MODE_TYPICAL",          0x0000007C);
define("CONF_SCAN_MODE_NAME",             0x0000007F);
// define("EXPRESS_SCAN_STABILITY_BITMAP",   4);
// define("EXPRESS_SCAN_SENSITIVITY_BITMAP", 5);
define("LEGACY_SAMPLE_DURATION", 476);

class Lidar
{
    public $_DEBUG = true;
    public $_log = "";

    protected $resource;
    protected $_isSupportingMotorCtrl = false;
    protected $_isConnected = false;
    protected $_motor_speed = DEFAULT_MOTOR_PWM;
    protected $_scanning = false;

    protected $_scan_size = 0;
    protected $_scan_anstype = ANS_TYPE_MEASUREMENT;
    protected $_buffer = "";

    protected $express_trame = 32;
    protected $express_data = false;
    
    protected $_ANS_TYPE = array(ANS_TYPE_MEASUREMENT                => "standard",
                                 ANS_TYPE_MEASUREMENT_CAPSULED       => "capsuled",
                                 ANS_TYPE_MEASUREMENT_HQ             => "HQ",
                                 ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA => "capsuled ultra",
                                 ANS_TYPE_MEASUREMENT_DENSE_CAPSULED => "dense capsuled");

    public function __construct($devfile = "/dev/ttyUSB0", $baudRate = 115200, $byteSize = 8, $parity = 'none')
    {
        $this->resource = new PhpSerial();
        $this->resource->deviceSet($devfile);
        $this->resource->confBaudRate($baudRate);
        $this->resource->confCharacterLength($byteSize);
        $this->resource->confParity($parity);
        $this->resource->confStopBits(1);
        $this->resource->confFlowControl("xon/xoff");
        $this->resource->deviceOpen();

        $this->_isConnected = true;
        $this->resource->serialflush();
        $this->_isSupportingMotorCtrl = $this->checkMotorCtrlSupport();
        $this->StopMotor();
    }

    private function _log($s)
    {
        if($this->_log=="")
        {
            print $s."\n";
        }
        else
        {
            file_put_contents($this->_log,date("Y-m-d H:i:s").": ".$s."\n");
        }
    }

    private function close()
    {
        $this->resource->deviceClose();
    }

    private function write($data)
    {
        if($this->_DEBUG) $this->_log("write to lidar = ".bin2hex($data)." ...");
        $this->resource->sendMessage($data);
    }

    private function read($count = 0)
    {
        $data = $this->resource->readPort($count);
        //if($this->_DEBUG) $this->_log("read from lidar = ".bin2hex($data)." ...");
        return $data;
    }


    private function setDtr($flag)
    {
        // not know how to set dtr  yet :(
        if($flag)
        {
            //$this->resource->
        }
        else 
        {
            //maybe worked :)
            if($this->_DEBUG) $this->_log("try stty!!!");
            $this->resource->_exec("stty -F ".$this->resource->_device." -hupcl");
        }
    }

    /**
     * Send request to lidar
     *      if payload defined calculate necessary size and control checksum
     *
     * @param int  $cmd - Command to send
     * @param string $payload - string with data bytes
     * 
     * @return void
     */
    private function _sendCommand($cmd, $payload = null/*, $payloadsize=0*/)
    {
        if (!$this->_isConnected)
        {
            throw new Exception('RESULT_OPERATION_FAIL');
        }
        $_req = chr(SYNC_S_BYTE) . chr($cmd);
        if(is_null($payload))
        {
            $this->write($_req);
        }
        else
        {
            $_checksum = 0;
            $_checksum ^= SYNC_S_BYTE;
            $_checksum ^= $cmd;
            $data = unpack('C*', $payload);
            $payloadsize = ( count($data)  & 0xFF);
            $_checksum ^= $payloadsize;
            $_req .= chr($payloadsize);
            for ($i = 1; $i <= $payloadsize; $i++) {
                $_checksum ^= $data[$i];
                $_req .= chr($data[$i]);
            }
            $_req .= chr($_checksum);
            $this->write($_req);
        }
    }

    /**
     * check supporting Motor control
     *      returns false for A1
     *      returns true for A2
     *      not known yet for S1
     *
     * @return bool 
     */
    public function checkMotorCtrlSupport()
    {
        $support = false;
        $flag = 0;
        $this->_sendCommand(CMD_GET_ACC_BOARD_FLAG, pack('L', $flag)); 
        try{
            $descriptor = $this->read_descriptor();
            if($descriptor["DSIZE"] != GET_ACC_BOARD_LEN)
            {
                throw new Exception('Wrong get_info reply length');
            }
            if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
            {
                throw new Exception('Not a single response mode');
            }
            if($descriptor["DTYPE"] != ANS_TYPE_ACC_BOARD_FLAG)
            {
                throw new Exception('Wrong response data type');
            }
            $rawdata = $this->read(GET_ACC_BOARD_LEN);
            $data = unpack("LFLAG",$rawdata);
            if($data["FLAG"])
            {
                $support = true;
            }
        } catch (Exception $e) {
            if($this->_DEBUG){
                echo $e->getMessage();
            };
        }
        return $support;
    }


    public function setMotorPWM($speed)
    {
        try{
            var_dump(pack("S", $speed));
            $this->_sendCommand(CMD_SET_MOTOR_PWM, pack("v", $speed));
            return true;
        } catch (Exception $e) {
            if($this->_DEBUG){
                echo $e->getMessage();
            };
            return false;
        }
    }

    public function startMotor()
    {
        if ($this->_isSupportingMotorCtrl) 
        { // RPLIDAR A2
            $this->setMotorPWM(DEFAULT_MOTOR_PWM);
            usleep(500000);
        }
        else
        { // RPLIDAR A1
            $this->setDTR(false);
            $this->setMotorPWM(DEFAULT_MOTOR_PWM);
            usleep(500000);
        }
    }
    
    public function stopMotor()
    {
        if ($this->_isSupportingMotorCtrl) 
        { // RPLIDAR A2
            $this->setMotorPWM(0);
            usleep(500000);
        } 
        else
        { // RPLIDAR A1
            $this->setDTR(true);
            usleep(500000);
        }
    }

    /**
     * Read descriptor of responsed data
     *
     * @return $array( string START_FLAG,     must be always in hex 0xA55A
     *                 long DSIZE,            size next 30bit in response
     *                 byte SEND_MODE,        next 2bit in response - mode: 0 - single, 1 - multi response, 2 and 3 - reserved for future 
     *                 byte DTYPE )           descriptor type
     */
    private function read_descriptor()
    {
        $descriptor = $this->read(DESCRIPTOR_LEN);
        if(strlen($descriptor) != DESCRIPTOR_LEN)
        {
            throw new Exception("Resource is not writable\n");
        }

        $data = unpack("A2START_FLAG/LDSIZE/CDTYPE",$descriptor);
        if($data["START_FLAG"] != chr(SYNC_S_BYTE) . chr(SYNC_R_BYTE))
        {
            throw new Exception('Incorrect descriptor starting bytes');
        }
        $data["DSIZE"] = $data["DSIZE"] & 0x3FFFFFFF;
        $data["SEND_MODE"] = $data["DSIZE"] >> 30;
        return $data;
    }

    /**
     * Exit the current state and enter the idle state
     *
     * @return boolean 
     * 
     */
    public function Stop()
    {
        try{
            $this->_sendCommand(CMD_STOP);
            usleep(1000);  // see the doc
            $this->_scanning = false;
            return true;
        } catch (Exception $e) {
            if($this->_DEBUG) $this->_log( $e->getMessage() );
            return false;
        }
    }

    /**
     * Reset (reboot) the RPLIDAR core
     * 
     * @return boolean 
     */
    public function Reset()
    {
        try{
            $this->_sendCommand(CMD_RESET);
            usleep(2000);  // see the doc
            $this->_scanning = false;
            return true;
        } catch (Exception $e) {
            if($this->_DEBUG) $this->_log( $e->getMessage() );
            return false;
        }
    }

    
    /**
     * Get info about Lidar
     *     returns array ( byte MODEL,            model, 24 - A1, 97 - S1
     *                     byte FW_MAJOR,         firmware major version
     *                     byte FW_MINOR,         firmware minor version
     *                     byte HARDWARE,         hardware, 7 - A1, 18 - S1
     *                     string SERIALNUMBER )
     * 
     * @return array 
     */
    public function getInfo()
    {
        $this->_sendCommand(CMD_GET_INFO);
        $descriptor = $this->read_descriptor();
        if($descriptor["DSIZE"] != GET_INFO_LEN)
        {
            throw new Exception('Wrong get_info reply length');
        }
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != ANS_TYPE_DEVINFO)
        {
            throw new Exception('Wrong response data type');
        }
        $rawdata = $this->read(GET_INFO_LEN);
        $data = unpack("CMODEL/CFW_MINOR/CFW_MAJOR/CHARDWARE/A16SERIALNUMBER",$rawdata);
        $data["SERIALNUMBER"] = bin2hex($data["SERIALNUMBER"]);
        return $data;
    }

    /**
     * Get health status from Lidar
     *     returns ( byte STATUS,   0 - Good, 1 - Warning, 2 - Error
     *                    int ERROR_CODE )
     * 
     * @return array 
     */
    public function getHealth()
    {
        $this->_sendCommand(CMD_GET_HEALTH);
        $descriptor = $this->read_descriptor();
        if($descriptor["DSIZE"] != GET_HEALTH_LEN)
        {
            throw new Exception('Wrong get_health reply length');
        }
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != ANS_TYPE_DEVHEALTH)
        {
            throw new Exception('Wrong response data type');
        }
        $rawdata = $this->read(GET_HEALTH_LEN);
        $data = unpack("CSTATUS/SERROR_CODE",$rawdata);
        return $data;
    }

    /**
     * Get single sampling time
     *     returns array ( STD  - time for take one sample in STANDARD mode in uS,
     *                     EXPR - time for take one sample in EXPRESS mode in uS)
     * @return array 
     */
    public function getSampleRate()
    {
        $this->_sendCommand(CMD_GET_SAMPLERATE);
        $descriptor = $this->read_descriptor();
        if($descriptor["DSIZE"] != GET_SAMPLERATE_LEN)
        {
            throw new Exception('Wrong get_health reply length');
        }
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != ANS_TYPE_SAMPLE_RATE)
        {
            throw new Exception('Wrong response data type');
        }
        $rawdata = $this->read(GET_SAMPLERATE_LEN);

        $data = unpack("SSTD/SEXPR",$rawdata);
        return $data;
    }

    /**
     * Get LIDAR configuration
     *
     * @param int $type - configuration entry
     * @param string $payload - additional payload
     * 
     * @return array $rawdata
     */
    private function getLidarConf($type, $payload=null)
    {
        $_req = pack("V", $type);
        if($payload != null)
        {
            //$_req .= pack("C*", $payload);
            $_req .= $payload;
        }
        $this->_sendCommand(CMD_GET_LIDAR_CONF, $_req);
        $descriptor = $this->read_descriptor();
        //var_dump($descriptor);
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != ANS_TYPE_GET_LIDAR_CONF)
        {
            throw new Exception('Wrong response data type');
        }
        $rawdata = $this->read($descriptor["DSIZE"]);
        $data = unpack("VTYPE/C*",$rawdata); 
        if($data["TYPE"] != $type)
        {
            throw new Exception('Returned type is NOT same as asked type');
        }
        //var_dump($data);
        return $data;
    }

    /**
     * Get sample duration of the scan mode, whose id is specified by the payload of request. 
     *
     * @param int $mode
     * 
     * @return float in microseseconds
     */
    public function getLidarSampleDuration($mode)
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_US_PER_SAMPLE, pack("v",$mode));
        $uS = (float)($d[1]/256+$d[2]+$d[3]*256+$d[4]*256*256);
        return $uS;
    }

    /**
     * Get max measurement distance of the scan mode, whose id is specified by the payload of request. 
     *
     * @param int $mode
     * 
     * @return float in meters
     */
    public function getMaxDistance($mode)
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_MAX_DISTANCE, pack("v",$mode));
        $max_d = (float)($d[1]/256+$d[2]+$d[3]*256+$d[4]*256*256);
        return $max_d;
    }

    /**
     * Get the answer command type of the scan mode, whose id is specified by the payload of request. 
     *
     * The return value is 8bit unsigned int, denotes the answer command type.  
     * Typical return answer types: 
     * 0x81 – For standard mode, returns data in rplidar_resp_measurement_node_t 
     * 0x82 – For express mode, returns data in capsuled format 
     * 0x83 – For boost, stability and sensitivity mode, returns data in ultra capsuled format 
     * 
     * @param int $mode
     * 
     * @return int
     */
    public function getScanModeAnsType($mode)
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_ANS_TYPE, pack("v",$mode));
        return $d[1];
    }

    /**
     * Get the name of scan mode, whose id is specified by the payload of request. 
     * The return value is a string of a user-friendly name for this scan mode.  
     *
     * @param int $mode
     * 
     * @return string
     */
    public function getScanModeName($mode)
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_NAME, pack("v",$mode));
        $s = "";
        for($i = 1; $i < count($d); $i++)
        {
            $s .= chr($d[$i]);
        }
        return $s;
    }

    /**
     *  Get the amount of scan modes supported by LIDAR
     * 
     *  RPLIDAR returns the amount of scan modes supported when receives this command.
     *    
     *  RPLIDAR supports scan mode ids from 0 to (scan_mode_count – 1). 
     *  For instance, device returning 2 according to this query means that the device 
     *  support 2 work modes, whose ids are 0, 1. The host system may use the work mode 
     *  id and other configuration type to get specific characters of the work mode. 
     *
     * @return int
     */
    public function getScanModeCount()
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_COUNT);
        $ret = $d[1]+$d[2]*256;
        return $ret;
    }


    /**
     * Get the typical scan mode id of LIDAR
     * no payload in request
     * must return uint 16
     *
     * @return int
    */
    public function getTypicalScanMode()
    {
        $d = $this->getLidarConf(CONF_SCAN_MODE_TYPICAL);
        $ret = $d[1]+$d[2]*256;
        return $ret;
    }


    /**
     * Get info about all Supported Scan modes
     *
     * @return array
     */
    public function getAllSupportedScanModes()
    {
        $info = $this->getInfo();
        $fw_ver = ($info["FW_MAJOR"]<<8) | $info["FW_MINOR"];
        if( $fw_ver >= 280) // 1.24 => 1<<8 + 24
        {
            $cnt = $this->getScanModeCount();
            $typical = $this->getTypicalScanMode();
    
            $modes = array();
            for($mode = 0; $mode < $cnt; $mode++)
            {
                $name = $this->getScanModeName($mode);
                $max_d = $this->getMaxDistance($mode);
                $uS = $this->getLidarSampleDuration($mode);
                $ans = $this->getScanModeAnsType($mode);
            
                $modes[$mode] = array("NAME" => $name,
                                      "MAX_DISTANSE" => $max_d,
                                      "SAMPLE_DURATION" => $uS,
                                      "ANSWER_TYPE" => $ans);
                if(isset($this->_ANS_TYPE[$ans])) $modes[$mode]["ANSWER_NAME"] = $this->_ANS_TYPE[$ans];
                if($mode == $typical) $modes[$mode]["TYPICAL"] = true;
            }
        }
        else
        {
            $modes[0] = array("NAME" => "Standard",
                              "SAMPLE_DURATION" => LEGACY_SAMPLE_DURATION,
                              "ANSWER_TYPE" => ANS_TYPE_MEASUREMENT,
                              "ANSWER_NAME" => $this->_ANS_TYPE[ANS_TYPE_MEASUREMENT]);
            if($fw_ver >= 273) // 1.17 => 1<<8 + 17
            {
                $d = $this->getSampleRate();
                $modes[0]["SAMPLE_DURATION"] = $d["STD"];
                $modes[1] = array("NAME" => "Express",
                                  "SAMPLE_DURATION" => $d["EXPR"],
                                  "ANSWER_TYPE" => ANS_TYPE_MEASUREMENT_CAPSULED,
                                  "ANSWER_NAME" => $this->_ANS_TYPE[ANS_TYPE_MEASUREMENT_CAPSULED]);
            }
        }
        return $modes;
    }






    /**
     * Start scanning
     *
     * @param int $ScanType
     * 
     * @return void
     */
    public function StartScan(int $ScanType=SCAN_TYPE_STANDARD)
    {
        //std
        // Command sent with payload: ['0xa5', '0xf0', '0x2', '0x94', '0x2', '0xc1']  -- CMD_SET_MOTOR_PWM = 660
        // Command sent: ['0xa5', '0x52']                                             -- CMD_GET_HEALTH
        // Received descriptor: ['0xa5', '0x5a', '0x3', '0x0', '0x0', '0x0', '0x6']   
        // Received data: ['0x0', '0x0', '0x0']                                       - read health
        // Command sent: ['0xa5', '0x20']                                             - start scan legacy
        // Received descriptor: ['0xa5', '0x5a', '0x5', '0x0', '0x0', '0x40', '0x81']   - read legacy start , дескриптор постоянный, 5 байт все время ответ

        // Received data: ['0x3e', '0xb1', '0x28', '0x0', '0x0']
        // Received data: ['0x3e', '0x55', '0x58', '0x0', '0x0']
        // Received data: ['0x3e', '0xd', '0x59', '0x14', '0x1c']

        //express
        // Command sent with payload: ['0xa5', '0xf0', '0x2', '0x94', '0x2', '0xc1'] -- CMD_SET_MOTOR_PWM = 660
        // Command sent: ['0xa5', '0x52']                                            -- CMD_GET_HEALTH
        // Received descriptor: ['0xa5', '0x5a', '0x3', '0x0', '0x0', '0x0', '0x6']
        // Received data: ['0x0', '0x0', '0x0']                                       - read health
        // Command sent with payload: ['0xa5', '0x82', '0x5', '0x0', '0x0', '0x0', '0x0', '0x0', '0x22']   - start scan expreaa
        // Received descriptor: ['0xa5', '0x5a', '0x54', '0x0', '0x0', '0x40', '0x82']                     -- дескриптор постоянный, 0x54 байт все время ответ?

        // Received data: ['0xaa', '0x5c', '0xcc', '0xd9', '0xb7', '0x20', '0x93', '0x20', '0xbb', '0x3', '0x0', '0x3b', '0x1e', '0xab', '0x3f', '0x1e', '0x3', '0x0', '0xbb', '0x2f', '0x20', '0x23', '0x20', '0xbb', '0x13', '0x20', '0x3', '0x20', '0xbb', '0xf3', '0x1f', '0xe7', '0x1f', '0xbb', '0xdf', '0x1f', '0xdb', '0x1f', '0xbb', '0xcb', '0x1f', '0xc3', '0x1f', '0xbb', '0xc3', '0x1f', '0xb7', '0x1f', '0xbb', '0xc7', '0x1f', '0xdf', '0x1f', '0xbb', '0xdb', '0x1f', '0xcf', '0x1f', '0xbb', '0xd3', '0x1f', '0xcf', '0x1f', '0xbb', '0xd3', '0x1f', '0xef', '0x1f', '0xbb', '0xf3', '0x1f', '0xf7', '0x1f', '0xbb', '0xb', '0x20', '0x7', '0x20', '0xbb', '0x0', '0x0', '0x0', '0x0', '0x0']
        // Received data: ['0xa2', '0x56', '0x8c', '0x5', '0x3', '0x0', '0x0', '0x0', '0x9', '0x3', '0x0', '0x3', '0x0', '0xaa', '0x3', '0x0', '0x3', '0x0', '0x99', '0x3', '0x0', '0x3', '0x0', '0x89', '0x3', '0x0', '0x3', '0x0', '0x87', '0x3', '0x0', '0x3', '0x0', '0x67', '0x3', '0x0', '0x3', '0x0', '0x66', '0x3', '0x0', '0x3', '0x0', '0x56', '0x3', '0x0', '0x3', '0x0', '0x45', '0x3f', '0xe', '0xcf', '0xd', '0x33', '0x6b', '0xd', '0x7', '0xd', '0x22', '0xa7', '0xc', '0x4f', '0xc', '0x12', '0x3', '0xc', '0xbb', '0xb', '0x12', '0x6f', '0xb', '0x27', '0xb', '0x11', '0xe6', '0xa', '0xaa', '0xa', '0xff', '0x6a', '0xa', '0x2', '0x0', '0x9f']
        

        $this->startMotor();
        $health = $this->getHealth();

        if($health["STATUS"] > 0)
        {
            $this->Reset();
            $health = $this->getHealth();
            if($health["STATUS"] > 0)
            {
                throw new Exception('RPLidar hardware failure! Error code: '.$health['ERROR_CODE']);
            }
        }

        switch($ScanType)
        {
            case SCAN_TYPE_STANDARD:
                $this->_sendCommand(CMD_SCAN);
                $this->_scan_size = MEASUREMENT_LEN;
                $this->_scan_anstype = ANS_TYPE_MEASUREMENT;
                break;

            case SCAN_TYPE_EXPRESS:
//                $this->_sendCommand(CMD_EXPRESS_SCAN, pack("a5",chr(0x0)));
                $this->_sendCommand(CMD_EXPRESS_SCAN, pack("x5"));
                $this->_scan_size = MEASUREMENT_EXPRESS_LEN;
                $this->_scan_anstype = ANS_TYPE_MEASUREMENT_CAPSULED;
                break;

            default:
                throw new Exception('Wrong/unknown scan type');
        }

        try
        {
            $descriptor = $this->read_descriptor();
            var_dump($descriptor);
            if($descriptor["DSIZE"] != $this->_scan_size)
            {
                throw new Exception('Wrong get_info reply length');
            }
            if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
            {
                throw new Exception('Not a single response mode');
            }
            if($descriptor["DTYPE"] != $this->_scan_anstype)
            {
                throw new Exception('Wrong response data type');
            }
            $this->_scanning = true;
        }
        catch (Exception $e)
        {
            if($this->_DEBUG) $this->_log( $e->getMessage() );
        }
        return true;
    }


    public function iter_scans(int $ScanType = SCAN_TYPE_STANDARD, $min_len=5)
    {
        $scan_list = [];
        foreach($this->iter_measures($ScanType) as $iterator)
        {
            if($iterator["new_scan"])
            {
                if(count($scan_list) > $min_len)
                {
                    yield $scan_list;
                }
                $scan_list = [];
            }
            if(($iterator["distance"] > 0) /* &&($iterator["quality"]==15) */)
            {
                $scan_list[] = array("angle" => $iterator["angle"],
                                     "distance" => $iterator["distance"] );
            }
        }
    }

    public function iter_measures(int $ScanType = SCAN_TYPE_STANDARD)
    {
        $this->startMotor();
        if (!$this->_scanning)
        {
            $this->StartScan($ScanType);
        }
        while(true)
        {
            $this->resource->deviceClose();
            $this->resource->deviceOpen("rb");
            //$this->_buffer = "";
            while(!feof($this->resource->_dHandle))
            {
                $buf = fread($this->resource->_dHandle, 8192);
                if($buf !== FALSE)
                {
                    yield from $this->_process_scan($buf, $ScanType);
                }
            }
            //$this->resource->serialflush();
            usleep(10000);
        }
    }


    private function _process_scan($rawdata, int $ScanType = SCAN_TYPE_STANDARD)
    {
        //echo bin2hex($rawdata)."\n";
        echo ".";
        $this->_buffer .= $rawdata;
        while(strlen($this->_buffer)>=$this->_scan_size)
        {
            $_scan_frame = substr($this->_buffer, 0, $this->_scan_size);
            if($this->_checkScanDataValid($_scan_frame, $ScanType))
            {
                echo "\t".bin2hex($_scan_frame)."\n"; 
                $this->_buffer = substr($this->_buffer, $this->_scan_size);
            } 
            else
            {
                $this->_buffer = substr($this->_buffer, 1);
                continue;
            }
            

            if($ScanType == SCAN_TYPE_STANDARD)
            {
                $capsule = $this->_parseStandardPacket($_scan_frame);
                if($capsule !== false)
                {
                    yield $capsule;
                } 
            }
            if($ScanType == SCAN_TYPE_EXPRESS)
            {
                if($this->express_trame == 32)
                {
                    $this->express_trame = 0;
                    if(!$this->express_data)
                    {
                        $this->express_data = $this->_parseExpressPacket($_scan_frame);
                        //$this->express_data = 
                    }
                    $this->express_old_data = $this->express_data;
                    $this->express_data = $this->_parseExpressPacket($_scan_frame);

                }
                $this->express_trame++;
                yield from $this->_process_express_frame($this->express_old_data, $this->express_data["start_angle"], $this->express_trame);

            }
        }
    }
    private function _checkScanDataValid($data, int $ScanType = SCAN_TYPE_STANDARD)
    {
        if($ScanType == SCAN_TYPE_STANDARD)
        {
            $new_scan = ord($data[0]) & 1;
            $inversed_new_scan = ((ord($data[0]) >> 1) & 1);
            $quality = ord($data[0]) >> 2;

            if($new_scan == $inversed_new_scan)
            {
                //throw new Exception('New scan flags mismatch');
                echo ' *** New scan flags mismatch *** ';
                return false;
            }
            $check_bit = ord($data[1]) & 1;
            if($check_bit != 1)
            {
                //throw new Exception('Check bit not equal to 1');
                echo ' *** Check bit not equal to 1 *** ';
                return false;
            }
            if($quality==15)
            {
                //echo json_encode($rec)."\n";
                return false;
            }    
        }
        if($ScanType == SCAN_TYPE_EXPRESS)
        {
            if( ( (ord($data[0]) >> 4) != 0xA ) or ( (ord($data[1]) >> 4) != 0x5 ) )
            {
                //throw new Exception('Cannot parse corrupted data');
//                echo ' *** Cannot parse corrupted data *** ';
                return false;
            }
            $checksum = 0;
            for($i=2; $i<strlen($data); $i++)
            {
                $checksum ^= ord($data[$i]);
            }
            if( $checksum != (ord($data[0]) & 0xF) + ((ord($data[1]) & 0xF) << 4) )
            {
                //throw new Exception('Invalid checksum');
//                echo ' *** Invalid checksum *** ';
                return false;
            }
        }
        return true;
    }

    private function _parseStandardPacket($data)
    {
        $new_scan = ord($data[0]) & 1;
        $angle = ((ord($data[1]) >> 1) + (ord($data[2]) << 7)) / 64.0;
        $distance = (ord($data[3]) + (ord($data[4]) << 8)) / 4.0;
        $rec = array("new_scan" => $new_scan,
//                     "quality" => $quality,
                     "angle" => $angle,
                     "distance" => $distance);
        // if($quality==15)
        if($angle<360)
        {
            //echo json_encode($rec)."\n";
            return $rec;
        }
        return false;
    }


    private function _parseExpressPacket($data)
    {
        $new_scan = ord($data[0]) >> 7;
        $start_angle = (ord($data[2]) + ((ord($data[3]) & 0x7F) << 8)) / 64;

        $measures = array();
        for($c = 0; $c<16; $c++)
        {
            $da1 = ord($data[4+$c*5]) << 8 + ord($data[5+$c*5]);
            $da2 = ord($data[6+$c*5]) << 8 + ord($data[7+$c*5]);
            $oa = ord($data[8+$c*5]);

            $distance1 = $da1 >> 2;
            $distance2 = $da2 >> 2;
            $delta1 = ( ($oa & 0xF) | (($da1 & 0x3)<<4));
            $delta2 = ( ($oa >> 4) | (($da2 & 0x3)<<4));

            $measures[] = array("distance" => $distance1, "delta" => $delta1);
            $measures[] = array("distance" => $distance2, "delta" => $delta2);
        }
        return array("new_scan" => $new_scan,
                     "start_angle" => $start_angle,
                     "measures" => $measures);

    }

    private function _process_express_frame($data, $new_angle, $frame)
    {
        $diffAngle = $new_angle - $data["start_angle"];
        if ($data["start_angle"] > $new_angle) {
            $diffAngle += 360;
        }
        $angle = $data["start_angle"] + $diffAngle*$frame/32 - $data["measures"][$frame]["delta"];
        $distance = $data["measures"][$frame]["distance"];

        $new_scan = ($new_angle < $data["start_angle"]) & ($frame == 1);
        // $angle = ($data["start_angle"] + ( ($new_angle - $data["start_angle"]) % 360)/32*$frame - $data["angle"][$frame-1]) % 360;
        // $distance = $data["distance"][$frame-1];
        return array("new_scan" => $new_scan, 
                     "angle" => $angle,
                     "distance" => $distance);
    }

}


?>