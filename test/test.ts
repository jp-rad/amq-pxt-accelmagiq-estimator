/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
input.onButtonPressed(Button.A, function () {
    accelmagiq.setCoordinateSystem(accelmagiq.CoordinateSystem.BASIC)
    basic.showLeds(`
        . . # . .
        . # # # .
        # . # . #
        . . # . .
        . . # . .
        `)
})
input.onButtonPressed(Button.AB, function () {
    input.calibrateCompass()
})
input.onButtonPressed(Button.B, function () {
    accelmagiq.setCoordinateSystem(accelmagiq.CoordinateSystem.TILT)
    basic.showLeds(`
        . . . . .
        . . # . .
        # # # # #
        . . # . .
        . . . . .
        `)
})
let quat: number[] = []
serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
serial.writeLine("*")
serial.writeLine("*")
serial.writeLine("*")
serial.writeLine("*")
serial.writeLine("AccelMagiQ")
basic.showLeds(`
    . # # # .
    # # . # .
    # . . # .
    # # # # #
    . . . # .
    `)
basic.forever(function () {
    quat = accelmagiq.estimate()
    serial.writeNumbers(quat)
    basic.pause(200)
})
