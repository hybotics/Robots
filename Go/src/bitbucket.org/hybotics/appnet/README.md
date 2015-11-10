appnet
======

Go interface to the App.net API - forked from github.com/whee/adn

Now known as bitbucket.com/hybotics/go-appnet

Installation
------------

The usual:

	$ go get bitbucket.org/hybotics/appnet

Package documentation
---------------------

/*
[godoc output courtesy of GoPkgDoc](http://go.pkgdoc.org/github.com/whee/adn)
*/

Example usage
-------------

getpost.go:

	package main

	import (
		"flag"
		"fmt"
		"bitbucket.org/hybotics/appnet"
		"log"
		"strconv"
	)

	var post = flag.Int("post", 1, "post id")

	func main() {
		flag.Parse()

		app := &adn.Application{}
		post, err := app.GetPost("", strconv.Itoa(*post)) // unauthenticated request
		if err != nil {
			log.Fatal(err)
		}

		fmt.Printf("%s [%v]\n%s\n", post.User.Username, post.CreatedAt, post.Text)
	}

Try it out:

	$ go run getpost.go -post=1
	mthurman [2012-08-03 03:59:06 +0000 UTC]
	join.app.net getting ready for the world w/ @dalton @berg @voidfiles @jhubball @aaronblyth @andrew @vinitlee @mark @mintz @barmstrong @laughingman @mikegreenspan @ben #joinus
