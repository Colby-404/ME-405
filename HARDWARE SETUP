# Assembly Notes

This section gives a step-by-step overview for assembling the Romi hardware stack, with emphasis on the **Shoe of Brian**, **controller stack**, and **bumper switch modules**.

---

# Shoe of Brian and Controller Stack Assembly

## 1. Verify the Shoe of Brian is modified

Before beginning assembly, confirm that the **ferrite bead has been removed** from the Shoe of Brian.

This modification is required so the Nucleo can be powered safely from the **Romi power distribution board** instead of through the Shoe's USB power path.

**Important notes:**
- Do **not** attach the Nucleo to the Shoe yet.
- Once the ferrite bead is removed, the Nucleo can no longer be powered normally through the Shoe's USB power path alone.
- During use, the robot must be powered by the **battery**, while USB is used for programming and communication.

---

## 2. Install the main chassis standoffs

Use:
- 4 × M2.5 x 30 mm standoffs
- 4 × M2.5 nylon lock nuts
- 5 mm nut driver
- small wrench or pliers

### Procedure
1. Locate the four mounting positions on the Romi chassis.
2. Insert the **30 mm standoffs upward from the top side** of the Romi chassis.
3. Secure each standoff with a nylon lock nut from below.
4. If a standoff location is a slot instead of a round hole, place the standoff at the **closed end of the slot**.
5. Tighten only until snug.

**Important:**  
Do **not** overtighten. The Romi chassis is ABS plastic and can be damaged by excessive torque.

---

## 3. Build and install the power, motor, and encoder cables

Before installing the upper plate and controller stack, prepare the cables that connect the Romi power distribution board to the Nucleo.

Each robot needs:
- 1 power cable
- 2 motor cables
- 2 encoder cables

These are typically made by re-pinning pre-crimped **Dupont-style jumper wires** into grouped housings.

### Recommended color convention

#### Encoder cable
| Contact | Color | Signal |
|--------|-------|--------|
| 1 | Blue | Encoder Channel A |
| 2 | Yellow | Encoder Channel B |

#### Motor cable
| Contact | Color | Signal |
|--------|-------|--------|
| 1 | Yellow | nSLP / Motor Enable |
| 2 | Blue | DIR / Motor Direction |
| 3 | Green | PWM / Motor Effort |

#### Power cable
| Contact | Color | Signal |
|--------|-------|--------|
| 1 | Black | GND |
| 2 | Red | VIN / Battery Power |

### Critical power warning
The battery power line must go to:

- **VIN**
- **NOT 5V**

An incorrectly wired power cable can destroy the Nucleo immediately when power is applied.

### Cable routing tips
- It is recommended to **twist the power cable wires together**
- Do **not** twist encoder or motor signal wires together unless you specifically need to
- Double-check connector polarity before plugging anything in

---

## 4. Install the acrylic Romi-to-Shoe adapter plate

Use:
- 4 × M2.5 x 10 mm screws
- 2 mm hex driver

### Procedure
1. Place the acrylic Romi-to-Shoe adapter on top of the four tall standoffs.
2. Secure the adapter plate with the four M2.5 x 10 mm screws.
3. Feed the two motor cables and two encoder cables through the **center cutout** in the adapter plate.

**Notes:**
- The adapter plate is symmetrical, so orientation does not matter.
- Do not overtighten, since the acrylic can crack.

---

## 5. Install the upper standoffs for the Shoe of Brian

Use:
- 4 × M2.5 x 8 mm standoffs

### Procedure
1. Thread the four 8 mm standoffs into the adapter plate.
2. Tighten carefully using a nut driver.

**Important:**
- Do not cross-thread the standoffs.
- Do not overtighten, or the acrylic threads may strip.

---

## 6. Mount the Shoe of Brian

Use:
- Shoe of Brian
- 4 × M2.5 x 6 mm screws
- 4 × nylon washers
- 2 mm hex driver

### Procedure
1. Place the Shoe of Brian on top of the 8 mm standoffs.
2. Put one nylon washer under each screw head.
3. Secure the Shoe of Brian to the standoffs using the four screws.
4. Orient the Shoe so the USB connector faces whichever direction your team prefers.
5. Route the motor and encoder cables between the standoffs so they exit near the correct Nucleo pins.

**Important notes:**
- The washers should sit **between the screw heads and the Shoe of Brian**
- Tighten only enough to slightly compress the washers
- Do not crush the washers or overtighten the screws

---

## 7. Reinstall the Nucleo onto the Shoe of Brian

### Procedure
1. Carefully align the Morpho headers on the Nucleo with the matching sockets on the Shoe of Brian.
2. Confirm that the USB ports on the Nucleo and Shoe are on the **same side**.
3. Press the Nucleo into place gently using finger pressure only between the two boards.

**Important:**
- Do not push through the Romi chassis or acrylic plate while mating the boards
- Do not “zipper” the Nucleo off the Shoe, since this can bend the Morpho pins

---

## 8. Connect all power, motor, and encoder cables

### Procedure
1. Connect the motor cables to their correct Nucleo pins.
2. Connect the encoder cables to their timer input pins.
3. Connect the power cable to:
   - **GND**
   - **VIN**
4. Verify every cable against the intended pinout before applying power.

### Final warning
Before turning power on:
- double-check every connector
- triple-check the power cable polarity
- verify that VIN is used instead of 5V

---

# IMU Assembly

## 1. Mount the BNO055 IMU

Use:
- BNO055 breakout board
- 4 × M2.5 x 10 mm standoffs
- 4 × M2.5 x 8 mm screws
- 4 × M2.5 nylon lock nuts
- 4 × nylon washers

### Procedure
1. Select the mounting holes on the Romi chassis for the IMU.
2. Install the standoffs to the IMU board first if that makes alignment easier.
3. Mount the IMU to the chassis using the screws, washers, and lock nuts.
4. Place the nylon washers between the screw heads and the IMU board.

### Recommended mounting note
It is often more convenient to mount the IMU **underneath the Romi chassis** rather than on top, since top mounting can interfere with front bumper hardware.

---

## 2. Build the IMU cable

The IMU cable should be made in the same way as the other custom Dupont cables.

### Suggested wiring
| IMU Pin | MCU Pin | Function |
|--------|---------|----------|
| SCL | PB13 | I2C clock |
| SDA | PB14 | I2C data |
| VCC | 3.3V or 5V | Power |
| GND | GND | Ground |

### Cable notes
- Keep **red** for power if possible
- Keep **black** for ground if possible
- Make the cable long enough to route cleanly, but not so long that it gets caught in the drivetrain or bump hardware

---

# Bumper Switch Assembly

## 1. Decide left or right before soldering

Each bumper switch PCB can be assembled as either a **left** or **right** module.

The handedness depends on:
- which side of the PCB faces upward
- which side receives the header pins

**Important:**  
Once the header is soldered in place, the module is permanently configured as either left or right.

For a full front bumper setup, use:
- 1 left module
- 1 right module

Together, the two modules support up to **six total bumper switches** across the front of the robot.

---

## 2. Solder the switches and header

Each module uses:
- 3 snap-action switches with roller levers
- 1 header strip

### Recommended approach
1. Decide whether the board will be used on the **left** or **right** side.
2. Insert the three roller-lever switches into the PCB.
3. Insert the header into the correct side of the PCB for the intended handedness.
4. Solder the switches in place.
5. Solder the header pins.

### Wiring note
If you are **not** using the original TI-RSLK chassis board arrangement, it may be easier to:
- use a **straight 0.1 in male header**, or
- solder wires directly to the board

This is often more convenient for custom Romi/Nucleo wiring.

---

## 3. Understand the bumper outputs before wiring

Each switch output connects to a **normally open** switch contact.

That means:
- the signal should use a **pull-up resistor**, or
- the MCU input should have its **internal pull-up enabled**

### Output behavior
- **Not pressed** → signal reads HIGH
- **Pressed** → signal is pulled LOW

This is important when writing the bumper driver code.

---

## 4. Mount the bumper modules to the Romi chassis

Each bumper module includes mounting hardware:
- 2 × #2-56 7/16 in screws
- matching nuts

### Procedure
1. Position the bumper assembly along the front edge of the Romi chassis.
2. Align the bumper switches so the roller levers sit where they can contact obstacles first.
3. Use the mounting holes in the roller switches to fasten the assembly to the chassis.
4. Secure the module with the provided screws and nuts.
5. Repeat for the opposite side if installing both left and right modules.

### Important fit check
Before fully tightening:
- make sure the roller levers move freely
- confirm the switches are not blocked by the chassis
- confirm the bumper modules do not interfere with the IMU or any other front-mounted hardware

---

## 5. Wire the bumper signals to the microcontroller

Each module provides three bumper outputs, commonly labeled as separate bumper channels.

### Wiring guidance
1. Connect each bumper output to a digital input pin on the MCU.
2. Enable internal pull-ups in software, or add external pull-ups in hardware.
3. Connect ground as needed for the chosen wiring method.
4. Label the bumper channels clearly so left, center, and right impacts can be identified in software.

---

## 6. Optional output combining

If the project does not need to know **which exact switch** was pressed, multiple bumper outputs can be combined on the PCB.

This can reduce the number of MCU pins required.

### Tradeoff
- fewer input pins used
- less detailed collision information

In most cases, keeping all bumper outputs separate is better for robot behavior and debugging.

---

# Final Assembly Check

Before powering the robot:

- confirm the ferrite bead is removed from the Shoe of Brian
- confirm the Nucleo is fully seated on the Shoe
- confirm the power cable goes to **VIN**, not 5V
- confirm motor and encoder cables are plugged into the correct pins
- confirm bumper outputs are wired to digital inputs with pull-ups
- confirm the IMU cable is wired correctly
- confirm no cables can get caught in the wheels or drivetrain
- confirm the bumper modules move freely and are not blocked
- confirm all screws are snug but not overtightened
