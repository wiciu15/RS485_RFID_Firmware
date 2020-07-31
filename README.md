# RS485_RFID_Firmware
Firmware for RS485 RFID Reader based on MFRC522 and STM32F103.


RTOS-based software running 3 tasks:

-defaultTask is polling reader every 100ms for new UUID

-ReceiveDataTask is resumed when new UUID is present. Then sends 'PING' to base device and if it respons, sends newly read UUID. Reader waits for up to 8 seconds to receive byte containing status code.

-statusReporting Task handles onbard LED and buzzer to report device status to user.


RC522 driver is not my work.
