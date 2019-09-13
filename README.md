# balancing_robot
A balancing robot trying to become sentient using a Jetson Nano and an Arduino

## Body

The body is made of multiple 3d printed components and held together by nuts and bolts. The components were designed in FreeCAD, converted to STL mesh files, sliced to gcode files and printed. I will publish schematics and STL files. May also publish gcode files.

## Components

### Motors
* 2x NEMA 17 stepper motors: https://www.aliexpress.com/item/32855316300.html?spm=a2g0s.9042311.0.0.27424c4dqhugXl
* 2x NEMA 17 steel brackets: https://www.aliexpress.com/item/32812576981.html?spm=a2g0o.productlist.0.0.24353bf8R1bj0d&algo_pvid=abdb741e-21e2-400d-8d48-00ffc31d99e9&algo_expid=abdb741e-21e2-400d-8d48-00ffc31d99e9-3&btsid=35bcf08e-3cdb-4076-85cf-0978074d6764&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52

### Motor control
* 2x TB67S249FTG stepper motor drivers: https://www.pololu.com/product/3096

### Wheels
* Alternative A:
  * 2x Pololu 90mm wheels: https://www.pololu.com/product/1438
  * 2x Pololu universal mounting hubs for 5mm shaft and M3 screws: https://www.pololu.com/product/1998
* Alternative B (Using some of the components in alternative A):
  * 2x 5mm hex couplers: https://www.aliexpress.com/item/32959682575.html?spm=a2g0s.9042311.0.0.27424c4docg9sb
  * 3d print a plastic wheel compatible to the above hex coupler and mount the tires of the aforementioned Pololu 90mm wheels on them. This is what I did and I will send the STL file for those wheels.

I initially used alternative A but I found out it does not withstand the robot's loads very well (in my case sharp turns and collisions at 15 km/h) so I moved to alternative B. 

### Microcontrollers and computers
* 1x Fake Arduino Nano board: https://www.aliexpress.com/item/32607801066.html?spm=a2g0s.9042311.0.0.27424c4dqhugXl
* 1x NVIDIA Jetson Nano board: https://www.amazon.com/NVIDIA-Jetson-Nano-Developer-Kit/dp/B07PZHBDKT

### Energy
* 2x battery holders, each for three 18650 li-ion batteries: https://www.aliexpress.com/item/32847696147.html?spm=a2g0s.9042311.0.0.27424c4dqhugXl
* 6x 18650 batteries (Sanyo NCR18650GA 3500mAh 10A batteries): https://www.aliexpress.com/item/32383140026.html?spm=a2g0s.9042311.0.0.61594c4dmH0ZFx
* 1x D24V10F9 9v 1A switching step down voltage regulator: https://www.pololu.com/product/2833
* 1x D24V50F5 5V 5A switching step down voltage regulator: https://www.pololu.com/product/2851

### IMU
* 1x MPU9250 IMU: https://www.aliexpress.com/item/32830071268.html?spm=a2g0s.9042311.0.0.27424c4d3Cm7zj

### Remote control
* 1x HC-05 bluetooth module: https://www.aliexpress.com/item/32806048234.html?spm=a2g0s.9042311.0.0.27424c4d3Cm7zj

### Miscellaneous
* 1x logic level converter: https://www.aliexpress.com/item/32851503557.html?spm=a2g0s.9042311.0.0.27424c4docg9sb

## Notes on components
* NEMA 17 stepper motors: If you can't obtain those motors you can, of course, try different NEMA 17 motors. I highly reccomend ones with a low resistance and a current rating of at most 2A. Even though a high resistance motor may draw significantly less current (though also a higher voltage), which may prevent the drivers from overheating, such a motor will not retain a high torque at high speeds. The motors I used are rated at 2.2V and 2A and therefore have a resistance of 1.1 Ohm which is very low for such motors.
* Arduino Nano board: Of course you can buy an original, but for me a 2$ fake does everything I need.
* TB67S249FTG stepper motor drivers: Those drivers are relatively expensive because they are Pololu drivers and have the feature of automatically lowering the current consumption of the motors when not needed. A generic chinese A4988 or DRV8835 will definitely do the work, though you may need to wire them differently and change the microstepping settings in the firmware. Link to the suggested alternative drivers: https://www.aliexpress.com/item/32963690420.html?spm=a2g0s.9042311.0.0.27424c4docg9sb
* Battery holders: In order to have 24V input voltage I daisy chained the two battery holders. I don't think it's a good idea even though it worked well up until now. Moreover, if you will need to recharge you will have to manually remove all the batteries and put them in a charger. Therefore, I instead recommend buying a 6s1p battery pack with a BMS instead. I plan to make a battery pack of my own (or buy one) and replace this hack.
* 18650 batteries: Of course you can use other batteries. Just beware of counterfeits and make sure they can deliver at least 5A of continuous current. I measured the capacity of the batteries I bought from the link I gave and indeed their capacity was 3500 mAh so I recommend buying from them.

