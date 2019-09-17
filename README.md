# balancing_robot
A balancing robot trying to become sentient using a Jetson Nano and an Arduino

## Body

The body is made of multiple 3d printed components and held together by nuts and bolts. The components were designed in FreeCAD, converted to STL mesh files, sliced to gcode files and printed.

### Components

Below is a list of the body components printed in a 3d printer. Each component has an stl file in the 3d_models directory.

- 2x side bars (SideBar.stl)
- 5x horizontal beams (HorizontalBeamV4.stl)
- 2x motor bracket connectors (MotorBracketConnectorV5.stl)
- 1x plate for holding the batteries (BatteriesPlate.stl)
- 1x plate for holding the Jetson Nano board (JetsonNanoPlate.stl)
- 1x plate for holding the main board (MainBoardPlateV2.stl)
- 1x bracket for the voltage/current meter (MeterHolder.stl)
- 1x camera holder (CameraHolder.stl)
- (optional) 2x custom pololu 90mm wheels, one is a mirrored version of the other (Pololu90mmWheel2.stl, Pololu90mmWheel2Mirrored.stl)

### Assembly

All components except a few can be assembled using M3 nuts and bolts. The custom Pololu wheels need M4 screws with a length of at least 8mm.

I tried to make the components as modular as possible. That's why I bombarded the side bars and horizontal bars with a 3.5mm diameter slots so it will be possible to mount a wide variety of objects in a wide variety of places on the robot.

Instructions (photos will come):

Skeleton:
- (2x side bars <- 2x horizontal beams) Connect two horizontal beams to the lowest part of the bottom most slots of the side bars.
- (2x side bars <- 2x horizontal beams) Connect two horizontal beams to the lowest part of the second from bottom slots of the side bars.
- (2x side bars <- 1x horizontal beam) Connect one horizontal beam to the lowest part of the upper most slot in the back of the side bar.

Motors:
- (2x side bars, 2x horizontal beams <- 2x motor bracket connectors) Connect the two motor bracket connectors to the side bars and horizontal beams. 

Electronics:
- (2x horizontal beams <- batteries plate) Connect batteries plate to the middle of the two lower horizontal beams.
- (2x horizontal beams <- main board plate) Connect main board plate to the right side of the two middle horizontal beams.
- (2x horizontal beams <- jetson nano plate) Connect jetson nano plate to the left side of the two middle horizontal beams.
- Camera and voltage meter: I mounted the camera to the middle of the forward-middle horizontal beam and the meter in the left side, but you can try different positions.

## Hardware

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

### Camera

* Waveshare 160-degree FOV camera with an IMX219 sensor: https://www.waveshare.com/imx219-160-camera.htm

### Miscellaneous
* 1x logic level converter: https://www.aliexpress.com/item/32851503557.html?spm=a2g0s.9042311.0.0.27424c4docg9sb
* 1x voltage and current meter: https://www.aliexpress.com/item/32824062417.html?spm=a2g0o.productlist.0.0.41297d3b2hvniV&algo_pvid=84a409ea-4096-4331-85cf-b470d72a05e7&algo_expid=84a409ea-4096-4331-85cf-b470d72a05e7-0&btsid=e9c926be-b88a-42ee-b7e7-672877d23d78&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52

## Notes on hardware
* NEMA 17 stepper motors: If you can't obtain those motors you can, of course, try different NEMA 17 motors. I highly reccomend ones with a low resistance and a current rating of at most 2A. Even though a high resistance motor may draw significantly less current (though also a higher voltage), which may prevent the drivers from overheating, such a motor will not retain a high torque at high speeds. The motors I used are rated at 2.2V and 2A and therefore have a resistance of 1.1 Ohm which is very low for such motors.
* Arduino Nano board: Of course you can buy an original, but for me a 2$ fake does everything I need.
* TB67S249FTG stepper motor drivers: Those drivers are relatively expensive because they are Pololu drivers and have the feature of automatically lowering the current consumption of the motors when not needed. A generic chinese A4988 or DRV8835 will definitely do the work, though you may need to wire them differently and change the microstepping settings in the firmware. Link to the suggested alternative drivers: https://www.aliexpress.com/item/32963690420.html?spm=a2g0s.9042311.0.0.27424c4docg9sb
* Battery holders: In order to have 24V input voltage I daisy chained the two battery holders. I don't think it's a good idea even though it worked well up until now. Moreover, if you will need to recharge you will have to manually remove all the batteries and put them in a charger. Therefore, I instead recommend buying a 6s1p battery pack with a BMS instead. I plan to make a battery pack of my own (or buy one) and replace this hack.
* 18650 batteries: Of course you can use other batteries. Just beware of counterfeits and make sure they can deliver at least 5A of continuous current. I measured the capacity of the batteries I bought from the link I gave and indeed their capacity was 3500 mAh so I recommend buying from them.
* While the camera noted above is supported by the Jetson Nano board, there are many cameras (and possibly other devices) that are not supported by the Jetson Nano. One example of an unsupported device is a V1 Raspberry PI camera (based on an ov5647 sensor) - it will simply not work (unless you want to compile the ov5647 driver as a loadable kernel module...). Therefore, when trying new devices double check that the Jetson Nano supports them.

## Wiring

TODO

## Software

### Low level controller (Arduino Nano)

#### Inputs and outputs
The low level controller's main responsibility is to balance the robot given the user's input (if there is any). In the current implementation the user inputs are:

* linear velocity
* turn rate
* online parameter tuning (will be discussed later)
* emergency stop
* linear velocity remote control sensitivity

Given those inputs, the low level controller controls the velocities of the two motors.

#### Block diagram
![low_level_controller](https://github.com/yinondouchan/balancing_robot/blob/master/arduino_block_diagram.png "Low level controller")

### High level controller (Jetson Nano)

Currently, the high level controller is a mix of experimental and proof-of-concept level software pieces. Yes, it sounds bad, but I will eventually organize it to something more coherent. The currently implemented software pieces are:

* Camera streaming - streams the video from the CSI camera to a webpage
* Follow me - detects and tracks an Aruco marker and makes the robot follow this marker by using the low level controller.
* trt-yolo-video - A TensorRT implementation of the YOLO detector (either V2 or V3, tiny and not tiny, can be selected in configuration) which takes the CSI camera's video as input and optionally outputs the video alongside with the detected objects to a webpage
* trt-goturn - A TensorRT accelerated GOTURN tracker. Almost works, but not yet!

