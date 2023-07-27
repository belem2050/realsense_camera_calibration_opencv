## Camera Calibration
Pyton script for calibrating realsense INtel camera by using CV2

### Installation
INtall depensdencies from rquirements.txt
```bash
pip install -r requirements
```
## Execution
 Run the following script:
```bash
python3 calibration.py --images_acquisition {TrueorFalse} --number_or_images {number} --period_of_capture {time[s]} --serial_number {TrueorFalse}
```
#### Parameters:
| command              | description                                     |default value
|:--------------------:|:-----------------------------------------------:|:-------------
|-h, --help            | show help and exit                              | -
|--images_acquisition  | set if want to acquire images                   | True
|--number_or_images    | the number of images to captures                | 10
|--period_of_capture   | The delay time between 2 captures               |  2
|--serial_number       | givethe serial number to upload cmears parameters|
--------------------------------------------------------------------------

## Camera calibrated parameters uploding
### Tool installation

Install librscalibrationtool by following :
1. ```bash
   apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE ```
2. ```bash
   add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u ```
3. ```bash
   sudo apt-get update && apt-get install -y --no-install-recommends librscalibrationtool ```
   
### Parameters uploading
To upload  with the serial number and the .xml file updated with the new obtained parameters 
1. ```bash
    /usr/bin/Intel.Realsense.CustomRW -sn {serial_number} --write --file {parameters}```
 To check if upload succeeded
 
1. ```bash
    /usr/bin/Intel.Realsense.CustomRW -sn {serial_number} -r ```
    

## License
Coalescent Mobile Robotics

   [Robot Framework]: <https://robotframework.org/>
