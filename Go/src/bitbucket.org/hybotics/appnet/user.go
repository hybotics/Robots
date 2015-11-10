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

import (
	"errors"
	"fmt"
	"time"
)

type User struct {
	Id          string      `json:"id"`
	Username    string      `json:"username"`
	Name        string      `json:"name"` // The user-supplied Name may be a pseudonym.
	Description Description `json:"description"`
	Timezone    string      `json:"timezone"`     // The Timezone is in tzinfo format.
	Locale      string      `json:"locale"`       // The Locale is in ISO format.
	AvatarImage Image       `json:"avatar_image"` // The URL and original size of the user's avatar.
	CoverImage  Image       `json:"cover_image"`  // The URL and original size of the user's cover image.
	Type        string      `json:"type"`         // An account can be human, bot, corporate, or feed.
	CreatedAt   time.Time   `json:"created_at"`   // The time at which the User was created in ISO 8601 format.
	Counts      Counts      `json:"counts"`
	AppData     interface{} `json:"app_data"`    // TODO: Opaque information an application may store.
	FollowsYou  bool        `json:"follows_you"` // Does this user follow the user making the request? May be omitted if this is not an authenticated request.
	YouFollow   bool        `json:"you_follow"`  // Does this user making the request follow this user? May be omitted if this is not an authenticated request.
	YouMuted    bool        `json:"you_muted"`   // Has the user making the request blocked this user? May be omitted if this is not an authenticated request.
}

type Description struct {
	Text     string   `json:"text"` // Biographical information
	HTML     string   `json:"html"` // Server-generated annotated HTML version of Text.
	Entities Entities `json:"entities"`
}

type Image struct {
	Height int    `json:"height"`
	Width  int    `json:"width"`
	Url    string `json:"url"`
}

type Counts struct {
	Following int `json:"following"` // The number of users this user is following.
	Followers int `json:"followers"` // The number of users following this user.
	Posts     int `json:"posts"`     // The number of posts created by this user.
	Stars     int `json:"stars"`     // The number of posts starred by this user.
}

/*
	Retrieve the user specified by idname.

	** Works **
*/
func (c *Application) GetUser(idname string) (u User, err error) {
	var epArgs EpArgs

	type GetUserResponse struct {
		Meta responseMeta `json:"meta"`
		Data User `json:"data"`
	}

	req := &Request {Token: c.Token}

	epArgs.User = idname

	resp := &GetUserResponse {}

	err = Do(req, "retrieve user", epArgs, resp)

	if err != nil {
		return
	}

    if resp.Meta.Code != 200 {
        txt := fmt.Sprintf("API Error: Code = %d, Message = %s", resp.Meta.Code, resp.Meta.ErrorMessage)
        err = errors.New(txt)
        return
    }

    u = resp.Data

	return
}

// Calls GetUser on the DefaultApplication.
func GetUser(idname string) (User, error) {
	return DefaultApplication.GetUser(idname)
}
