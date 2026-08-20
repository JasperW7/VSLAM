## For HackClub Macondo

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
- gears x1
- pulleys x1
  
The build was designed mainly around M3 screws, and the control system revolves around a [Rev Control Hub](https://www.revrobotics.com/rev-31-1595/), 12V battery (xt30 end), 4 standard 25T PWM servos, 25T servo horns and a [C270 Logitech webcam]( https://www.logitech.com/en-ca/shop/p/c270-hd-webcam).
For the sake of time, we printed our own [688 bearings](https://grabcad.com/library/688-bearing-8x16x5mm-1), however metal 688 bearings will work as well.

For assembly, simply attach the servos to the frame & gears with screws, then mount the wheels after. This avoids the wheels from blocking the screw mounting for the servos. 
  
