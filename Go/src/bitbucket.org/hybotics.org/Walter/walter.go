package main

import (
  "time"

  "github.com/hybridgroup/gobot"
  "github.com/hybridgroup/gobot/platforms/firmata"
  "github.com/hybridgroup/gobot/platforms/gpio"
)

func main() {
  walter := gobot.NewGobot()

  firmataAdaptor := firmata.NewFirmataAdaptor("arduino", "/dev/tty.usbmodem411")
  led := gpio.NewLedDriver(firmataAdaptor, "led", "13")

  work := func() {
    gobot.Every(1*time.Second, func() {
      led.Toggle()
    })
  }

  walter.AddRobot(gobot.NewRobot("W.A.L.T.E.R.",
    []gobot.Connection{firmataAdaptor},
    []gobot.Device{led},
    work,
  ))

  walter.Start()
}
