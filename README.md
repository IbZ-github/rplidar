# rplidar

library for working Slamtec RPLIDAR
php version - limited functionality
python version - extended by new method

tested with A1 and S1

Example output for S1
<pre>
LIDAR Info: {'model': 97, 'firmware': (1, 28), 'hardware': 18, 'serialnumber': '7E11EAF2C5E19BCFC2E19FF589C34509'}
LIDAR Health status: ('Good', 0)
LIDAR Motor sontrol support: False
Legacy sample rate: (244, 108)
Scan modes count = 2
mode #0, name:Standard, answer type: 0x81, max distance: 40.0, sample duration: 244.0
mode #1(* typical), name:DenseBoost, answer type: 0x85, max distance: 40.0, sample duration: 108.0
</pre>

Example output for A1
<pre>
LIDAR Info: {'model': 24, 'firmware': (1, 29), 'hardware': 7, 'serialnumber': 'BE569A86C0E09CC7A2E09DF72C843077'}
LIDAR Health status: ('Good', 0)
LIDAR Motor sontrol support: False
Legacy sample rate: (508, 254)
Scan modes count = 5
mode #0, name:Standard, answer type: 0x81, max distance: 12.0, sample duration: 508.0
mode #1, name:Express, answer type: 0x82, max distance: 12.0, sample duration: 254.0
mode #2, name:Boost, answer type: 0x84, max distance: 12.0, sample duration: 127.0
mode #3(* typical), name:Sensitivity, answer type: 0x84, max distance: 12.0, sample duration: 127.0
mode #4, name:Stability, answer type: 0x84, max distance: 12.0, sample duration: 201.0
</pre>