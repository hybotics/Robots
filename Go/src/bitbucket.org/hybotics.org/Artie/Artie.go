package main

import (
	"time"
	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/firmata"
	"github.com/hybridgroup/gobot/platforms/gpio"
	"bitbucket.org/hybotics/appnet"
)

func main() {
	artie := gobot.NewGobot()

	firmataAdaptor := firmata.NewFirmataAdaptor("arduino", "/dev/tty.usbmodem411")
	led := gpio.NewLedDriver(firmataAdaptor, "led", "13")

	work := func() {
		gobot.Every(1*time.Second, func() {
			led.Toggle()
		})
	}

	artie.AddRobot(gobot.NewRobot("Artie",
		[]gobot.Connection{firmataAdaptor},
		[]gobot.Device{led},
		work,
	))

	artie.Start()
}