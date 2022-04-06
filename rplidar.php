<?php

include 'PhpSerial.php';

define("SYNC_S_BYTE", "\xA5");
define("SYNC_R_BYTE", "\x5A");

define("CMD_STOP", "\x25");
define("CMD_RESET", "\x40");
define("CMD_GET_INFO", "\x50");
define("CMD_GET_HEALTH", "\x52");
define("CMD_GET_SAMPLERATE", "\x59");       //added in fw 1.17

define("CMD_EXPRESS_SCAN", "\x82");         //added in fw 1.17
define("CMD_HQ_SCAN", "\x83");              //added in fw 1.24
define("CMD_GET_LIDAR_CONF", "\x84");       //added in fw 1.24
define("CMD_SET_LIDAR_CONF", "\x85");       //added in fw 1.24

//add for A2 to set RPLIDAR motor pwm when using accessory board
define("CMD_SET_MOTOR_PWM", "\xF0");
define("CMD_GET_ACC_BOARD_FLAG", "\xFF");

define("MAX_MOTOR_PWM",  1023);
define("DEFAULT_MOTOR_PWM", 660);


// Response
define("SEND_MODE_SINGLE", 0);
define("SEND_MODE_MULTI", 1);
define("SEND_MODE_RSRVD1", 2);
define("SEND_MODE_RSRVD2", 3);

#define RPLIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x82
#define RPLIDAR_ANS_TYPE_MEASUREMENT_HQ            0x83

// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_SAMPLE_RATE      0x15
//added in FW ver 1.23alpha
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA  0x84
//added in FW ver 1.24
#define RPLIDAR_ANS_TYPE_GET_LIDAR_CONF     0x20
#define RPLIDAR_ANS_TYPE_SET_LIDAR_CONF     0x21
#define RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED        0x85
#define RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

define("DESCRIPTOR_LEN", 7);
define("GET_INFO_LEN", 20);
define("GET_INFO_TYPE", 4);
define("GET_HEALTH_LEN", 3);
define("GET_HEALTH_TYPE", 6);


define("SCAN_TYPE_STANDARD", 0);
define("SCAN_TYPE_EXPRESS", 1);
define("SCAN_TYPE_BOOST", 2);
define("SCAN_TYPE_SENSITIVITY", 3);
define("SCAN_TYPE_STABILITY", 4);
define("SCAN_TYPE_DENSE_BOOST", 5);


/*
#   define RPLIDAR_CONF_SCAN_COMMAND_STD            0
#   define RPLIDAR_CONF_SCAN_COMMAND_EXPRESS        1
#   define RPLIDAR_CONF_SCAN_COMMAND_HQ             2
#   define RPLIDAR_CONF_SCAN_COMMAND_BOOST          3
#   define RPLIDAR_CONF_SCAN_COMMAND_STABILITY      4
#   define RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY    5

#define RPLIDAR_CONF_ANGLE_RANGE                    0x00000000
#define RPLIDAR_CONF_DESIRED_ROT_FREQ               0x00000001
#define RPLIDAR_CONF_SCAN_COMMAND_BITMAP            0x00000002
#define RPLIDAR_CONF_MIN_ROT_FREQ                   0x00000004
#define RPLIDAR_CONF_MAX_ROT_FREQ                   0x00000005
#define RPLIDAR_CONF_MAX_DISTANCE                   0x00000060
        
#define RPLIDAR_CONF_SCAN_MODE_COUNT                0x00000070
#define RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE        0x00000071
#define RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE         0x00000074
#define RPLIDAR_CONF_SCAN_MODE_ANS_TYPE             0x00000075
#define RPLIDAR_CONF_SCAN_MODE_TYPICAL              0x0000007C
#define RPLIDAR_CONF_SCAN_MODE_NAME                 0x0000007F
#define RPLIDAR_EXPRESS_SCAN_STABILITY_BITMAP                 4
#define RPLIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP               5
*/

class Lidar
{
    protected $resource;
    protected $_DEBUG = false;
    //protected $_motor_speed = DEFAULT_MOTOR_PWM
    //protected $scanning = [False, 0, 'normal']
    //protected $express_trame = 32
    //protected $express_data = false;
    //protected $motor_running = null;


    public function __construct($devfile = "/dev/ttyUSB0", $baudRate = 115200, $byteSize = 8, $parity = 'none')
    {
        $this->resource = new PhpSerial();
        $this->resource->deviceSet($devfile);
        $this->resource->confBaudRate($baudRate);
        $this->resource->confCharacterLength($byteSize);
        $this->resource->confParity($parity);
        $this->resource->deviceOpen();
    }

    private function close()
    {
        $this->resource->deviceClose();
    }

    private function write($data)
    {
        $this->resource->sendMessage($data);
    }

    private function read($count = 0)
    {
        return $this->resource->readPort($count);
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
            throw new Exception('Resource is not writable');
        }

        $data = unpack("A2START_FLAG/LDSIZE/CDTYPE",$descriptor);
        if($data["START_FLAG"] != SYNC_S_BYTE . SYNC_R_BYTE)
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
            $this->write(SYNC_S_BYTE . CMD_STOP);
            usleep(1000);  // see the doc
            return true;
        } catch (Exception $e) {
            if($this->_DEBUG){
                echo $e->getMessage();
            };
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
            $this->write(SYNC_S_BYTE . CMD_RESET);
            usleep(2000);  // see the doc
            return true;
        } catch (Exception $e) {
            if($this->_DEBUG){
                echo $e->getMessage();
            };
            return false;
        }
    }

    /**
     * Undocumented function
     *
     * @param int $ScanType
     * 
     * @return void
     */
    public function StartScan(int $ScanType=SCAN_TYPE_STANDARD)
    {
        return true;
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
        $this->write(SYNC_S_BYTE . CMD_GET_INFO);
        $descriptor = $this->read_descriptor();
        if($descriptor["DSIZE"] != GET_INFO_LEN)
        {
            throw new Exception('Wrong get_info reply length');
        }
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != GET_INFO_TYPE)
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
        $this->write(SYNC_S_BYTE . CMD_GET_HEALTH);
        $descriptor = $this->read_descriptor();
        if($descriptor["DSIZE"] != GET_HEALTH_LEN)
        {
            throw new Exception('Wrong get_health reply length');
        }
        if($descriptor["SEND_MODE"] != SEND_MODE_SINGLE)
        {
            throw new Exception('Not a single response mode');
        }
        if($descriptor["DTYPE"] != GET_HEALTH_TYPE)
        {
            throw new Exception('Wrong response data type');
        }
        $rawdata = $this->read(GET_HEALTH_LEN);
        $data = unpack("CSTATUS/SERROR_CODE",$rawdata);
        return $data;
    }

    /**
     * Get single sampling time
     */
    public function getSampleRate()
    {
        //
    }

    /**
     * 
     * 
     */
    /**
     * Get LIDAR configuration
     *
     * @param int $type - configuration entry
     * @return void
     */
    public function getLidarConf($type)
    {
        //
    }

}


?>