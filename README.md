# Hardware Wrappers
Hardware wrappers for FRC parts, controllers, &amp; encoders

Designed to provide easy sim real mixing by supporting simulation of your entire robot through a model

## Use
Hop on over to build.gradle inside of repositories add
```gradle
maven {
        url = uri("https://maven.pkg.github.com/NE-Robotics/wrappers")
        credentials {
            username = "Mechanical-Advantage-Bot"
            password = "\u0067\u0068\u0070\u005f\u006e\u0056\u0051\u006a\u0055\u004f\u004c\u0061\u0079\u0066\u006e\u0078\u006e\u0037\u0051\u0049\u0054\u0042\u0032\u004c\u004a\u006d\u0055\u0070\u0073\u0031\u006d\u0037\u004c\u005a\u0030\u0076\u0062\u0070\u0063\u0051"
        }
}
```
and inside of dependencies add
```gradle
implementation 'com.northeasternrobotics.wrappers:wrappers:0.0.1'
```

## Supported Hardware
#### Encoders
- CANCoder
- SRXEncoder
- ThriftyEncoder
- SimSwerveAzimuthEncoder

#### Gyros
- ADXRS453
- NavX
- SimGyro

#### Motor Controllers
- SparkMax
- TalonFX
- TalonSRX
- Venom
- SimSmartMotor

### Note
yes I'm borrowing Mechanical Advantage's bot so anyone can use this, you can also put your own username and a token with package read privileges if you would prefer
