# pca9685_hardware_interface

## Gestion des erreurs I²C

Les méthodes `ReadRegisterByte` et `WriteRegisterByte` de `PiPCA9685::I2CPeripheral` relaient désormais fidèlement les codes d'erreur du noyau Linux en utilisant `errno` lorsque l'accès SMBus échoue. Les messages d'exception incluent à la fois un contexte sur le registre ciblé et la chaîne système correspondant à l'`errno`, ce qui facilite le diagnostic des erreurs matérielles (par exemple `ENXIO` pour une adresse invalide).

Un test (`i2c_peripheral_error_test`) simule un périphérique I²C inexistant et vérifie que l'exception lève bien le code `ENXIO` et mentionne l'erreur dans le message.
