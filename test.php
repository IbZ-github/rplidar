#!/usr/bin/php

<?php

include "rplidar.php";

$lidar = new Lidar();

// $d = $lidar->getInfo();
// print "      MODEL: ".$d["MODEL"]."\n";
// print "   FIRMWARE: ".$d["FW_MAJOR"].".".$d["FW_MINOR"]."\n";
// print "   HARDWARE: ".$d["HARDWARE"]."\n";
// print "     SERIAL: ".strtoupper($d["SERIALNUMBER"])."\n";

// $d = $lidar->getAllSupportedScanModes();
// print "+--+-----------+----------+------------+-----------+----------------+\n";
// print "|id|SCAN MODE  | SAMPLE uS|MAX DISTANCE|ANS TYPE ID|ANSWER TYPE NAME|\n";
// print "+--+-----------+----------+------------+-----------+----------------+\n";
// foreach($d as $mode => $info)
// {
//     if(isset($info["TYPICAL"]) && $info["TYPICAL"]) $mode = $mode."*";
//     printf("|%2s|%-12s|%9.2f |%11.2f |      0x%2X |%-16s|\n", $mode, $info["NAME"], $info["SAMPLE_DURATION"], $info["MAX_DISTANSE"], $info["ANSWER_TYPE"], $info["ANSWER_NAME"]);
// }
// print "+--+-----------+----------+------------+-----------+----------------+\n";
// print "* - typical\n";

// $lidar->Reset();
// $lidar->Stop();
// exit;


$iter = 0;
foreach($lidar->iter_scans(SCAN_TYPE_EXPRESS) as $scan_data)
//foreach($lidar->iter_scans() as $scan_data)
{
    $iter++;
    print_r($scan_data);
    echo "\n\n!! Get measures count = ".count($scan_data)."\n\n";
    if($iter > 5)
    {
        break;
    }
}

?>