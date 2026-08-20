## For HackClub Macondo
*This repo is forked off of the official FTC SDK & Pedro Pathing
### Monocular VSLAM Turret
I was slightly involved in a project a while back focusing on drones and the use of camera vision for mapping over something like a laser used for rangefinding. I got interested in the idea of visual simultaneous localization & mapping (VSLAM) and how it could be integrated into drone navigation systems. My vision for this project was to mainly test out the system flow & software behind a single camera VSLAM (monocular), less so about the automation and navigation. Another challenge I wanted to tackle was configuring monocular VSLAM using a cheap and affordable webcam with low processing power.

### Finished product
Currently, the product is able to use OpenCV to return a camera stream back to Panels stream for viewing, as well as displaying the bot's pose in field coordinates (also viewable under Panels Field).
### Build
All STL Files can be found under [STL Folder](https://github.com/JasperW7/VSLAM/tree/main/STL) with a viewable [Onshape](https://cad.onshape.com/documents/fbfda43b3c9a34854064ec10/w/3fedeb5aa6c3b0ad226559e0/e/34f30c8dd9f021da3e9b9f1b)
You will need:
- wheel x2
- frame x1
- camera holder x1
- gears (2) x1
- pulleys (2) x1
  
The build was designed mainly around M3 screws, and the control system revolves around a [Rev Control Hub](https://www.revrobotics.com/rev-31-1595/), 12V battery (xt30 end), 4 standard 25T PWM servos, 25T servo horns, 64T belt, and a [C270 Logitech webcam]( https://www.logitech.com/en-ca/shop/p/c270-hd-webcam).
For the sake of time, we printed our own [688 bearings](https://grabcad.com/library/688-bearing-8x16x5mm-1), however metal 688 bearings will work as well.

For assembly, simply attach the servos to the frame & gears with screws, then mount the wheels after. This avoids the wheels from blocking the screw mounting for the servos. 
Next, attach the bearings into the frame and big gear holders (we used super glue). You can attach the big gear to the frame and the camera holder into the holders now. Attach the pulley to the tilt servo with a belt running across the pulleys.
Finally, remove the stock c270 mount from the webcam and slide the isolated webcam into the camera holder.
Wire the servos to ports 0,1,2, and 3, plug in the webcam usb to the USB port on the control hub and connect the battery to power it up.
<img width="2160" height="2880" alt="image" src="https://github.com/user-attachments/assets/702dd06e-b05b-4529-bd0b-d0f4d681265c" />

### Software
To get started with software, simply install Android Studio and clone this repository. To upload this repository onto the control hub, simply connect a usb c to the control hub port and click on the "Run TeamCode" button. Another option is to install the ADB Wifi plugin and connect to the control hub wifi to push code. 

To access the Panels Dashboard, connect to the robot wifi and head to http://192.168.43.1:8001/ on your browser. For setup, under the Teleop dropdown, select the calibration opmode. You will need to print out the following checkerboard pattern used for OpenCV's checkerboard calibration: https://github.com/opencv/opencv/blob/4.x/doc/pattern.png. Run the opmode and select widgets under the Panels Dashboard for Camera Stream. Ensure that the webcam is able to see the checkerboard and move the camera around (you can use the pan and tilt servos for this or you can move the camera manually). The calibration will select 20 points and return calibration values (fx, fy, etc.) to telemetry. Hold onto these values.

Finally, you can run the "test vslam" opmode. Under configurable variables, plug in the fx, fy, cx, cy, k1, etc. calibrated values. The defaults in there are the values that worked for me. Under Panel's camera stream, there should be points plotted by OpenCV's ORB feature extraction. For mapping, simply add a widget for the Panels Field. On top of the field drawing, there will be lines displayed for the camera's pose.

  
