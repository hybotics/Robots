/*
	appnet - Go package for the App.Net API.
	Copyright (C) 2013 Dale A. Weber <hybotics.pdx@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	Package:		appnet - Go package for accessing the App.Net API
	Date:			30-Mar-2014
	Version:		0.1.0

	Purpose:		To make the App.Net API accessible from the Go language
						
	Dependencies:	None

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.
*/

/*
	Original Copyright and Attribution:

	Copyright (c) 2012 Brian Hetro <whee@smaertness.net>
		Use of this source code is governed by the ISC
		license that can be found in the LICENSE file.
*/
package appnet

type Entities struct {
	Mentions []Mention
	Hashtags []Hashtag
	Links    []Link
}

type Mention struct {
	Name string `json:"name"` // The username being mentioned (not including '@').
	Id   string `json:"id"`   // The user id of the mentioned user.
	Pos  int    `json:"pos"`  // The 0-based index where the entity includes Text (includes '@').
	Len  int    `json:"len"`  // The length of the substring in Text.
}

type Hashtag struct {
	Name string `json:"name"` // The name of the hashtag (not including '@').
	Pos  int    `json:"pos"`  // The 0-based index where the entity includes Text (includes '#').
	Len  int    `json:"len"`  // The length of the substring in Text.
}

type Link struct {
	Text string `json:"text"` // The anchor text to be linked.
	Url  string `json:"url"`  // The destination url.
	Pos  int    `json:"pos"`  // The 0-based index where the entity begins in Text.
	Len  int    `json:"len"`  // The length of the substring in Text that represents this link.
}
