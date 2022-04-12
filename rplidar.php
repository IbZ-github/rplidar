<?php

include 'PhpSerial.php';

define("SYNC_S_BYTE", 0xA5);
define("SYNC_R_BYTE", 0x5A);

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

define("SCAN_TYPE_STANDARD",    0);
// define("SCAN_TYPE_EXPRESS",     1);
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

define("CONF_ANGLE_RANGE",                0x00000000);
define("CONF_DESIRED_ROT_FREQ",           0x00000001);
define("CONF_SCAN_COMMAND_BITMAP",        0x00000002);
define("CONF_MIN_ROT_FREQ",               0x00000004);
define("CONF_MAX_ROT_FREQ",               0x00000005);
define("CONF_MAX_DISTANCE",               0x00000060);
        
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
    protected $resource;
    protected $_DEBUG = false;
    protected $_isSupportingMotorCtrl = false;
    protected $_isConnected = false;
    protected $_motor_speed = DEFAULT_MOTOR_PWM;
    //protected $scanning = [False, 0, 'normal']
    //protected $express_trame = 32
    //protected $express_data = false;
    //protected $motor_running = null;

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
        $this->resource->deviceOpen();

        $this->_isConnected = true;
        $this->resource->serialflush();
        $this->_isSupportingMotorCtrl = $this->checkMotorCtrlSupport();
        $this->StopMotor();
    }

    private function close()
    {
        $this->resource->deviceClose();
    }

    private function write($data)
    {
        if($this->_DEBUG){
            print "\nwrite to lidar = ".bin2hex($data)." ...\n";
        }
        $this->resource->sendMessage($data);
    }

    private function read($count = 0)
    {
        return $this->resource->readPort($count);
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
    private function checkMotorCtrlSupport()
    {
        $support = false;
        $flag = 0;
        $this->_sendCommand(CMD_GET_ACC_BOARD_FLAG, pack('L', $flag)); //maybe V
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
            $this->setDTR(true);
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
            $this->_sendCommand(CMD_RESET);
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
        return true;
    }

}


?>