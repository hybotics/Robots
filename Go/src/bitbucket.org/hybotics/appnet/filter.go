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

type Filter struct {
	Type           string   `json:"type"`             // Either "show" or "block" for whether this filter should exclude everything except for what's shown or show everything except for what's blocked.
	Name           string   `json:"name"`             // A User assigned name for this filter.
	UserIds        []string `json:"user_ids"`         // A slice of user ids a Post must or must not be created by.
	Hashtags       []string `json:"hashtags"`         // A slice of hashtags a Post must or must not have.
	LinkDomains    []string `json:"link_domains"`     // A slice of domains a Post must or must not have a link to.
	MentionUserIds []string `json:"mention_user_ids"` // A slice of user ids a Post must or must not mention.
}
