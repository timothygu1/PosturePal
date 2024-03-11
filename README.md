# PosturePal

PosturePal is a wearable 'smart' posture corrector designed to enable proper sitting posture amongst scoliosis patients recovering from spinal fusion surgery.
This project was designed to operate using the STM32F401 Nucleo Board, with device behaviour coded using C/C++.

**Materials:** 
- STM32 microcontroller
- 5 potentiometers
- 6 custom 3D printed parts
- Piezo Buzzer.
  
![posturepal](https://github.com/timothygu1/PosturePal/assets/123818948/f35c3fb4-700a-4800-8904-383b4758b4f9)

### Features/Requirement Specifications:

- The device continuously monitors the angle of the user's spine from the pelvis to the neck. The spine should remain within a tolerance of 10 degrees in any direction to distribute spinal load evenly. If the user bends their spine past this threshold, the device will vibrate a Piezo buzzer to prompt them to return to the correct posture.

- The device not surpass 3.5 kilograms in weight [7]. This requirement is crucial, especially considering the device's intended use by children and teenagers during the recovery stage after surgery.

- For accurate reading of spinal angle, the potentiometers offer a resistance range of 0-10kΩ and exhibit an error range of 100 ohms in either direction. The circuit accurately reads the voltage drop across the potentiometers with an error that does not exceed 25 degrees and 100 ohms.

- To ensure safety and effectiveness, the device operates on a voltage supply of 3-5V DC. This voltage range is deemed safe for clothed electronics, avoiding potential safety hazards.


