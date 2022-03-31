#!/usr/bin/php

<?php

include "rplidar.php";

$port = new Lidar();

$d = $port->getInfo();
var_dump($d);
exit;

$port->write("\xa5"."\x50");
$read = $port->read();
$port->close();

$s = unpack("H*",$read);

var_dump($read);
var_dump($s);



?>