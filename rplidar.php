<?php

include 'PhpSerial.php';

define("SYNC_S_BYTE", "\xA5");
define("SYNC_R_BYTE", "\x5A");

define("GET_INFO_BYTE", "\x50");
define("GET_HEALTH_BYTE", "\x52");
define("STOP_BYTE", "\x25");
define("RESET_BYTE", "\x40");

define("DESCRIPTOR_LEN", 7);
define("GET_INFO_LEN", 20);
define("GET_INFO_TYPE", 4);
define("GET_HEALTH_LEN", 3);
define("GET_HEALTH_TYPE", 6);

define("SEND_MODE_SINGLE", 0);
define("SEND_MODE_MULTI", 1);
define("SEND_MODE_RSRVD1", 2);
define("SEND_MODE_RSRVD2", 3);



class Lidar
{
    protected $resource;
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
     * Get info about Lidar
     *
     * @param int $mode  Cut mode, either CUT_FULL or CUT_PARTIAL
     * @param int $lines Number of lines to feed
     *
     * @return $array( byte MODEL,            model, 24 - A1, 97 - S1
     *                 byte FW_MAJOR,         firmware major version
     *                 byte FW_MINOR,         firmware minor version
     *                 byte HARDWARE,         hardware, 7 - A1, 18 - S1
     *                 string SERIALNUMBER )
     */
    public function getInfo()
    {
        $this->write(SYNC_S_BYTE . GET_INFO_BYTE);
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
     *
     * @return $array( byte STATUS,   0 - Good, 1 - Warning, 2 - Error
     *                 int ERROR_CODE )
     */
    public function getHealth()
    {
        $this->write(SYNC_S_BYTE . GET_HEALTH_BYTE);
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

}


?>