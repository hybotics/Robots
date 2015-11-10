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
  Package:    appnet - Go package for accessing the App.Net API
  Date:     30-Mar-2014
  Version:    0.1.0

  Purpose:    To make the App.Net API accessible from the Go language
            
  Dependencies: None

  Comments:   Credit is given, where applicable, for code I did not originate.
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
	"encoding/json"
	"testing"
)

var testUserData = []byte(`
{
    "avatar_image":{
      "height":200,
      "url":"https:\/\/d2rfichhc2fb9n.cloudfront.net\/image\/4\/zBIezrirTTAVL1JQFy9mgA8RePruklFAwH9TNCqvR3fxoeGPC-JnYYN3yvkMT907ZvnrEA7HWt3Nk8kBBPmnwa78iRo3HArocVxx97_zjfL5nv-vVvjG-63zeQeSKH1iPdGuxPF70NBajBFy2qkCXGDX44A",
      "width":200
    },
    "counts":{
      "followers":5239,
      "following":455,
      "posts":3121,
      "stars":216
    },
    "cover_image":{
      "height":456,
      "url":"https:\/\/d2rfichhc2fb9n.cloudfront.net\/image\/4\/iV3sRFel7xEjDK7hCB9R0xgIFAH7fQEtmq_KmotfjaDfuOEPbfcy2go0g5_fees1lwanOhzqz-BNSyC9bF09pHQryBA-Gsi_3Z_Gz81IZMibsiFaZ4TajDFDhoBFxE6H4TGXJZxdW1ZGA0VmZQN0n6MsEdM",
      "width":1103
    },
    "created_at":"2012-08-03T01:17:14Z",
    "description":{
      "entities":{
        "hashtags":[
          
        ],
        "links":[
          {
            "len":7,
            "pos":12,
            "text":"App.net",
            "url":"http:\/\/App.net"
          },
          {
            "len":18,
            "pos":28,
            "text":"daltoncaldwell.com",
            "url":"http:\/\/daltoncaldwell.com"
          }
        ],
        "mentions":[
          
        ]
      },
      "html":"<span itemscope=\"https:\/\/app.net\/schemas\/Post\">Founder\/CEO <a href=\"http:\/\/App.net\">App.net<\/a> \r\nBlog: <a href=\"http:\/\/daltoncaldwell.com\">daltoncaldwell.com<\/a><\/span>",
      "text":"Founder\/CEO App.net \r\nBlog: daltoncaldwell.com"
    },
    "follows_you":false,
    "id":"1",
    "is_follower":true,
    "is_following":false,
    "is_muted":false,
    "locale":"en_US",
    "name":"Dalton Caldwell",
    "timezone":"America\/Los_Angeles",
    "type":"human",
    "username":"dalton",
    "you_follow":true,
    "you_muted":false
}`)

func TestUserDecode(t *testing.T) {
	u := &User{}
	if err := json.Unmarshal(testUserData, u); err != nil {
		t.Error("json.Unmarshal(testUserData)", err)
	}
}

func TestUserGet(t *testing.T) {
	// Not authenticated. This should fail.
	_, err := GetUser("", "@whee")
	if err == nil {
		t.Error("unauthenticated GetUser() should have failed")
	}
}
