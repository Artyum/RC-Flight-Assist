# RC Flight Assist (RCFA) Telemetry System

I present to you a one-of-a-kind telemetry system for RC pilots. The RC Flight Assist (RCFA) is designed specifically for outdoor pattern flying, such as F3A or F3M classes. The system allows better control over the airplane during acrobatics, implementing a radio telemetry mechanism that indicates the position inside the flight area with high accuracy. With RCFA, pilots can maintain the proper position in the sky, resulting in better handling and improved skills. The system was tested in collaboration with the best Polish F3A pilot of 2015.

#### How Does It Work?

The RCFA system comprises two separate electronic devices: the Transmitter and the Receiver. The primary role of the Transmitter is to get the 3D or 2D position from GPS and altimeter sensors and send the data to the pilot’s Receiver. The Receiver converts the signal into sound patterns, which the pilot can hear through the built-in speaker or headphones to receive telemetry information.

The basic usage of the RCFA involves defining two points on the ground to determine the flight area. The first point (P1) is the pilot’s position, where the airplane will be controlled from. The second point (P2) is the direction to the flight line where the airplane shall be controlled. These points can be set by walking with the RCFA to the desired positions on the airfield or by writing the coordinates in the configuration file (rcfa.ini). The optimal distance between P1 and P2 depends on GPS signal strength and usually does not need to be more than 20m.

#### Setting Up the Flight Area

Once points P1 and P2 are set within the Transmitter, the RCFA system automatically calculates the remaining flight area coordinates based on preset options: the ideal Flight Line (marked with points A, B, and C), the Green Zone (green color), and the Flight Area (blue color). The pilot’s job is to keep the airplane between the green rectangles.

During a flight, the RCFA Receiver generates tones that change according to the airplane’s distance from the Green Zone. This allows the pilot to react quickly and steer the airplane to the desired position in the sky. The pilot will know exactly how far the airplane is from the flight line, and whether it is ahead or behind the Green Zone, too high, or too far to the left or right side of the Flight Area.

#### Post-Flight Scoring

When the flight is over, the system calculates a score based on the time spent in the zone and overall flight time. This score is automatically stored in a KML file. The logger file can be opened instantly in Google Earth (Windows or Linux) to examine the flight and check the scoring.

#### Additional Features of the RCFA

**GPS Logger**: The system is highly customizable. Users can change various parameters such as the distance from the pilot to the flight line, area angle, flight plane width, and more (nearly 30 options). It has an integrated MicroSD slot for a memory card, which is used to store and read area parameters and to write position data as a high-accuracy GPS Logger. The system implements unique filter algorithms, ensuring high accuracy in the GPS data stream and altitude measurements. The RCFA also allows the use of a barometer for even more accurate altitude measurement. The Logger writes data directly in KML format, which can be instantly opened in Google Earth.

**Real-Time Tracking**: The RCFA can stream data to the Receiver about current 3D coordinates through the RX/TX interface (USB2TTL). The data stream (longitude, latitude, and altitude) can be read and visualized on a PC in real-time. Depending on barometer availability, the altitude data can be taken from GPS or barometer. Users can choose either absolute (mean sea level) or relative (to the pilot’s position) altitude readings.

**Variometer**: The RCFA includes a variometer feature, which can be used in gliders and other types of RC airplanes. The variometer can use GPS data or a barometer to indicate altitude changes. The data is sent to the Receiver, allowing the pilot to control descending or ascending in real-time.

# Overview
https://www.youtube.com/watch?v=m0nCrscLPIs

#### RC Flight Assist - Real-time tracking demo
https://www.youtube.com/watch?v=_wvuzFEu3uk

## Full Features List

The list of configurable parameters:
* Distance in meters between the Pilot and the Flight Line
* Area angle in degrees
* Flight Area width
* Flight line width
* Flight Area boundary alarm (Out-Of-Area)
* Altitude alarm
* Set points P1 and P2 manually
* Automatic Flight Area lock
* Real-time tracking
* Automatic reference altitude
* Manual setting of reference altitude
* GPS sample rate in 'Reads Per Second'
* GPS Logger Automatic mode
* GPS Logger Manual mode (free flight)
* Logger speed frequency
* Four different modes of Floor option
* "Low accuracy" information feature
* Configurable filtering algorithm for GPS, Altimeter, and Variometer
* Radio module switch
* Altimeter switch
* Variometer mode
* (Variometer) Minimum vertical speed ratio
* (Variometer) Maximum vertical speed ratio
* Time zone relative to UTC
* Initial sound volume in the Receiver
* Event log

### GPS Logger

RC Flight Assist can be used as a GPS Logger. The data is stored on a Micro SD card in KML format, which can be opened directly in Google Earth. The flight path is visualized as a line in 3D space as an animated track. The logger is controlled by several options:

1. **Stream Speed Frequency**: The coordinates can be saved every 0.5 to 60 seconds.
2. **GPS Data Acquiring Speed**: The data can be acquired 1, 5, or 10 times per second.
3. **Reference Altitude**: You can manually set the accurate reference altitude to ensure the plot is at the exact height. The altitude measurement relies on the high-accuracy barometer sensor MPL3115A2. If the barometer is not used, the altitude data is acquired from GPS.

The Logger can work in two modes:

**Automatic Mode**: Designed for acrobatic flights, so the pilot does not have to remember to enable or disable the Logger before each flight. The logger will start when the airplane flies into the flight zone and stop when leaving it.

**Manual Mode**: Along with the attached barometer, this makes RCFA a high-quality GPS Logger. In this mode, the Logger must be enabled and disabled manually by pressing the relevant buttons on the transmitter. To reduce energy drain, the radio module can be turned off.

### Real-time Tracking

The RC Flight Assist (RCFA) system is capable of sending position data to the Receiver in real-time. This data stream can be visualized on a PC directly in Google Earth. Currently, Google Earth has limited capabilities for displaying altitude in real-time mode. However, several third-party applications, such as GooPs (goopstechnologies.com) or EarthBridge (mboffin.com/earthbridge), can work as extensions to Google Earth to enhance its real-time tracking capabilities.

### Technical Details

**Radio Frequency**: 433MHz, 868MHz, 915MHz  
**Frequency-Hopping Spread Spectrum (FHSS)**  
**Range**: 700+/-100m (tested in open air with copper wire antenna)  
**Input Voltage**: 7-12V (2S/3S LiPo)  
**GPS Compatibility**: Global Top (MT3318, MT3329, MT3339), u-Blox (M7, M8)

#### Receiver (RX)
* **Headphones**: 3.5mm jack port
* **Average Current**: 50mA
* **Size**: 51 x 38mm
* **Weight**: ~17g

#### Transmitter (TX)
* **MicroSD Card**: 256MB-32GB class 4 (FAT32)
* **Maximum Transmit Power**: 20dB (100mW)
* **Average Current**: 150mA
* **Size**: 52 x 40mm

### Variometer

The RCFA Telemetry System can function as a variometer, providing real-time altitude data sourced from the GPS module or barometer (if available). Barometric altitude readings generally offer higher quality, but certain GPS modules such as u-Blox M8 or M7 series can also provide excellent 3D positioning signals. This telemetry system enables pilots to control ascents and descents in real-time from the ground. Specific sound patterns emitted by the receiver indicate altitude changes.

#### Customization Options

The variometer mode is highly customizable:
- **Low Pass**: Defines the minimum altitude change considered as horizontal flight.
- **High Pass**: Sets the altitude change threshold that triggers sound gradation in the speaker.

This flexibility allows pilots to tailor the variometer's responsiveness to their specific flying needs and preferences.

## Receiver
![RCFA_RX](https://github.com/user-attachments/assets/d5b933c2-fd32-4f24-8eff-beaaea0ff7ff)

## Transmitter
![RCFA_TX](https://github.com/user-attachments/assets/2efa2826-e206-46d4-8a16-0e7089b3a6cc)

## Kit
![IMG_20160701_214035_HDR](https://github.com/user-attachments/assets/719ba96b-8cad-41c0-9516-e0149fd7bca6)
