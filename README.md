## For HackClub Macondo
*This repo is forked off of the official FTC SDK & Pedro Pathing

*Heavy inspiration from https://learnopencv.com/monocular-slam-in-python/
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
Wire the servos to ports 0,1,2, and 3, plug in the webcam usb to the USB port on the control hub and connect the battery to power it up. [Wiring Diagram](https://github.com/JasperW7/VSLAM/blob/main/Wiring%20Diagram.png)
<img width="2160" height="2880" alt="image" src="https://github.com/user-attachments/assets/702dd06e-b05b-4529-bd0b-d0f4d681265c" />

### Software
To get started with software, simply install Android Studio and clone this repository. To upload this repository onto the control hub, simply connect a usb c to the control hub port and click on the "Run TeamCode" button. Another option is to install the ADB Wifi plugin and connect to the control hub wifi to push code. 

To access the Panels Dashboard, connect to the robot wifi and head to http://192.168.43.1:8001/ on your browser. For setup, under the Teleop dropdown, select the calibration opmode. You will need to print out the following checkerboard pattern used for OpenCV's checkerboard calibration: https://github.com/opencv/opencv/blob/4.x/doc/pattern.png. Run the opmode and select widgets under the Panels Dashboard for Camera Stream. Ensure that the webcam is able to see the checkerboard and move the camera around (you can use the pan and tilt servos for this or you can move the camera manually). The calibration will select 20 points and return calibration values (fx, fy, etc.) to telemetry. Hold onto these values.

Finally, you can run the "test vslam" opmode. Under configurable variables, plug in the fx, fy, cx, cy, k1, etc. calibrated values. The defaults in there are the values that worked for me. Under Panel's camera stream, there should be points plotted by OpenCV's ORB feature extraction. For mapping, simply add a widget for the Panels Field. On top of the field drawing, there will be lines displayed for the camera's pose.
<img width="1919" height="667" alt="image" src="https://github.com/user-attachments/assets/19dae7e3-39d6-461e-9ceb-8100892896a0" /> 
___
## Day-by-day journal entries (Failures, Thoughts, etc.) [https://docs.google.com/document/d/1HC7wofTFJbAAgc9IIZ6yAzwW5cy-wWwRABX2THkkn_8/edit?tab=t.0]

Aug 2

Our vision for this project is to learn and utilize Visual Simultaneous Localization & Mapping (VSLAM) using a single cheap and affordable webcam. Our goal is to have a fully autonomous 2 degree of freedom camera scan the full area in 3-d space and map out the environment accurately. 

Today, we focused on planning out our design and taking inventory of the parts we had available. For our project, we wanted to incorporate multiple subsystems that worked together, namely a 2-wheel drive base, a 2DOF turret and a camera. 

In our first sketch, we discussed how our robot would behave in the environment. For example, our initial idea was to only include a panning turret, however we decided that we wanted to have a robot that was able to drive around and a secondary tilt in addition to the panning. We drew rough ideas of how these mechanisms would interact with each other, such as the tilt servo driving a belt that controls the shaft that the camera is mounted onto.

After our rough draft, we spent some time detailing our drawing to define the major parts of the build. Notably this includes the implementation of our servos and how we plan for them to provide the 2 axis of rotation. This allows us to more thoroughly visualize the project before we begin our CAD.

Aug 2 

With a general idea of the direction we were headed, we began CADing on OnShape. While I (Jasper) imported the CAD of the REV servo and generated the gears, Bowen began modeling the servo mount/frame. 

We wanted as many parts of our robot to be as 3d-printable as possible, however, one issue we encountered was the possibility of printing servo horns. Since the spline was too fine, we opted for buying aluminum horns. One issue we faced was finding online STL/Step files that were available to match the online listings so that we could accurately place them into our CAD. We worried that if a cad didn’t match, parts wouldn’t mesh properly. Eventually, we found a proper step file and based all of the servo attachments such as the wheels and gears off of it.

Designing the servo mount was relatively smooth as it was just the projection of the servos’ outline to create holes. 2 mounts were created vertically for the wheels and one was created horizontally on the surface of the mount for the camera’s panning. We had to tweak the positioning of the servo as initially we wished for the servo to be placed vertically (shown in the following picture), though it ended up taking too much space. This was not a difficult change fortunately as we just rolled back a couple edits and recreated the necessary holes.


Another issue we forgot to account for was the existence of the control system. We forgot to incorporate the control hub and battery into our original sketch, and since we wanted the profile of the robot to be as small as possible, another challenge was fitting the control system in without taking up too much space. We found that the space between the wheel servos was enough to fit the control hub and we could still attach the battery underneath as long as we extended the frame. We intend to add a balancing ball at the bottom to keep the battery from touching the ground. 

Aug 3 

With the foundation of the bot finished, today we aim to model the camera mount and the pulley system that will control the camera’s tilt. Going off of our sketch, we had a few major parts to CAD. The large gear has to be modified with handles that support the shaft and pulley system (Jasper). A hex shaft also has to be made with an integrated mount which will ultimately hold the camera (Bowen).

Handles

The handles were relatively smooth to CAD. Just 2 extrusions on the gear with a rounded top so the camera clears it when looking up and down. Going into this project, we wanted to create this robot without the need for funding by using spare parts we already own. Jasper already has a pulley system of 2 gears and a belt (we may just union the gear on the shaft to the shaft itself to print as one part, but we’ll consider that later). The belt he has is 64 teeth, which caused some issues in the modeling of the handles as we needed to continually increase the height of them to accommodate for its length. This was also necessary as the camera must sit high enough to capture the floor in its FOV without being blocked by the chassis. Additionally, a slot for our fourth and final servo that will be spinning the servo.

Shaft/Camera Mount

The shaft went through a couple iterations in its design as the cameras’ hinges and dimensions were a tad finicky. I (Bowen) initially created the shaft to be a cylinder rather than a hex shaft. This was not very practical as there needs to be as much friction as possible for it to move with the bearing. 

We are using a logitech camera that has this mounting hole on the back:

So I initially designed the axle to have a horn that threads through the hole to support the camera vertically (1).  This small horn was very brittle and would’ve likely snapped due to the weight of the camera or the motion of the pulley. As such, I added a back to the horn that would help to hold up the camera. While I was designing this, it was about the time when Jasper imported the CAD for his pulley and mated it onto our existing model. It was then that I noticed the camera indeed clipped into the pulley as I didn’t expect the pulley to be so large (2). This is why we ended up rotating the camera, placing it horizontally to help it clear the pulley. I then restarted the design for my axle to account for this, making sure the lens itself was still centered and the camera protruded enough to clear the pulley (3).

Since most of the CAD was near completion, we decided to go through and check if there were any failure points that could occur, including checking how wires would route and fasteners. 

We decided to perform voronoi pocketing on the frame to create holes where we could zip tie extra lengths of wire so they wouldn’t drag on the ground. This came with the extra benefit of aesthetics as well while not being damaging to structural integrity. 

Another problem we realized was that the servo we were using for panning the gears was blocking the USB port on the control hub, meaning the camera wouldn’t be able to connect to the control system. Because of this, we rotated the servo around the spline 180 degrees to instead block the HDMI port. We also added in the balancing ball point underneath the battery box to hold it up against the ground. We figured that a point round surface scraping the ground would be better than an entire flat surface against the ground. 

Finally, after much review, we sent the necessary CAD pieces off to print (while we go out to buy fasteners). 

We also started looking into an article about the algorithm behind monocular VSLAM and understanding how it works, previewing ourselves for when we get into the software side.



We looked inside multiple hardware stores for M3 screws but couldn’t find any. Nonetheless, we decided to first stop and test the controller we would use to control the robot. We wanted to test the controllers connection to the driver station APK and ensure that it would work. 

One issue we found was that we couldn’t directly connect the controller straight to the phone with a usb c to c wire. For connections, the controller would only return values through a usb A wire. Therefore, we bought an adapter to go from usb c to usb a to usb c :). This worked :).

We picked up the parts for our control system including the control hub, adapter wire and logitech c270 webcam that we would be using. (We were also going to pickup our CAD prints too but the print layer shifted so we restarted the print). 

One thing we noticed when we tested the battery to wire converter to control hub power was the connection was iffy. If the wire was bent the wrong way, the power would disconnect. Seeing as the power had to go through a tamiya connector and then to an xt30 connector, we imagined that the tamiya connection was a potential failure point. So, we cut off the tamiya connection and soldered the xt30 head straight to the wire.


While soldering, we had trouble getting the solder wire to flow through the wire. We thought flux would allow it to flow better, but even with flux, the solder wire just wouldn’t flow. This is when we realized that better solder wire is more advantageous than just good flux alone. The solder wire we got was bought cheap online, which explains why it wouldn’t flow properly. Even so, after soldering, our wire passed the pull test so we figured it would hold up. 


We tested it by connecting it to the control hub and it wouldn’t power on. We thought it was a soldering mistake again so we re-soldered the joint. After the second attempt was finished, it still wouldn’t power on. This is when we realized that the fuse that is inside the battery wire had been blown out. Once we swapped the fuse, it powered up.

## **Assembly**

Though the battery can connect to the control hub, we forgot the charging wire connected through the tamiya head. As such, we had to spend some time soldering an adapter that connects the battery’s xt30 back to the tamiya to be able to charge.

### Bearings
As the prints were finished we began to assemble the chassis. We ran into a couple issues with the bearings and the shafts they were supposed to hold. It seems that the tolerance for the shaft was incorrectly done. It took a lot of force to get the shaft into the bearing. In doing so we broke the axle under the big gear.

After it superglues back on, we’ll sand or file it down so it cleanly fits into the bearing. The camera shaft has a similar issue, but we got it in eventually. All the holes for the bearings were a bit too large and made them loose, though it's not much of an issue as we can use superglue to keep them in place.

## Camera Mount
The shaft that holds the camera also did not fit the camera in real life. More specifically the backing that was supposed to hold the camera ended up being so long that the rod in the middle could not be threaded into the hole in the middle. We had to cut off a bit of the edge which allowed the camera to fit.

![image](https://cdn.hackclub.com/019fd9f4-a027-7ea0-8d80-873272b1eeb1/image.png)

## Battery holder
A similar thing happened with the battery holder. The battery was bigger in real life, so we just cut off a side of the holder to help it squeeze in.

![image](https://cdn.hackclub.com/019fd9f3-a6c2-73fb-b6cb-4b5bdda003c6/image.png)

## Pulley Installation of Doom
There were many issues with the pulley installation. Aside from the bearings not fitting the shaft perfectly, we forgot to put the belt in before we put the axle on. So we had to remove the already stuck and glued bearings and put the belt in. Then we realised the shaft itself was put on the wrong way (not on the side of the servo), so we had to take everything off AGAIN and flip it. Yikes!

The screws we used to connect the wheels to the servo were too long, so we cut them to size.

![image](https://cdn.hackclub.com/019fd9f6-56dc-7f6c-b8fe-389174918ec4/image.png)

## Axle on the Big Gear
Back to the axle on the big gear that snapped off when we tried forcing the bearing in. The superglue was taking a while to dry, so we put a bit of water around it. For some reason it turned white and still didn’t dry. Not sure how we’re going to deal with this but I hope it’ll just dry eventually.

![image](https://cdn.hackclub.com/019fd9f4-29ce-78c8-906c-ede3ea2bc133/image.png)

Here's some final pictures of our progress today:
![image](https://cdn.hackclub.com/019fd9f8-1a60-7a19-96f4-04e79ef1a697/image.png)

