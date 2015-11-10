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

type Post struct {
	Id           string    `json:"id"`
	User         User      `json:"user"`       // Embedded version of the User object. May not exist if the account has since been deleted.
//	CreatedAt    string    `json:"created_at"` // The time at which the post was created.
	CreatedAt    time.Time `json:"created_at"` // The time at which the post was created.
	Text         string    `json:"text"`       // User supplied text of the post.
	HTML         string    `json:"html"`       // Server-generated annotated HTML rendering of the post text.
	Source       Source    `json:"source"`
	ReplyTo      string    `json:"reply_to"`      // Id of the post this post is replying to (or "" if not a reply).
	ThreadId     string    `json:"thread_id"`     // Id of the post at the root of the thread this post is a part of.
	CanonicalURL string    `json:"canonical_url"` // The URL of the post's detail page on Alpha.

	NumReplies int `json:"num_replies"` // The number of posts created in reply to this post.
	NumStars   int `json:"num_stars"`   // The number of users who have starred this post.
	NumReposts int `json:"num_reposts"` // The number of users who have reposted this post.

	Annotations []interface{} `json:"annotations"` // TODO implement

	Entities Entities `json:"entities"`

	IsDeleted   bool `json:"is_deleted"`   // Has this post been deleted?
	MachineOnly bool `json:"machine_only"` // Is this post meant for machines or humans?

	YouStarred bool     `json:"you_starred"` // Has the current user starred this post?
	StarredBy  []string `json:"starred_by"`  // A partial list of users who have starred this post. (TODO verify)

	YouReposted bool     `json:"you_reposted"` // Has the current user reposted this post?
	Reposters   []string `json:"reposters"`    // A partial list of users who have reposted this post. (TODO verify)
	RepostOf    *Post    `json:"repost_of"`    // If this post is a repost, the original post. (TODO verify self-reference)
}

type Source struct {
	Name string `json:"name"` // Description of the API consumer that created this post.
	Link string `json:"link"` // Link provided by the API consumer that created this post.
}

// Retrieve the post specified by id using token as authentication.
func (c *Application) GetPost (id string) (p Post, err error) {
	var epArgs EpArgs

	type Response struct {
		Meta ResponseMeta `json:"meta"`
		Data Post 		  `json:"data"`
		Error string
	}

	resp := &Response {}
	req := &Request {
		Token: c.Token,
	}

	epArgs.Post = id

	err = Do(req, "retrieve post", epArgs, resp)

	if err != nil {
		etxt := fmt.Sprintf("appnet.post.GetPost/%s/%s", c.Name, err)
		err = errors.New(etxt)
		return
	}

	meta := resp.Meta

	if meta.Code != 200 {
		etxt := fmt.Sprintf("appnet.post.GetPost/%s/API Error! Code: %d, Message: %s, ID: %s", c.Name, meta.Code, meta.ErrorMessage, meta.ErrorId)
		err = errors.New(etxt)	
		return
	}

	if resp.Error != "" {
		etxt := fmt.Sprintf("appnet.post.GetPost/%s/%s", c.Name, resp.Error)
		err = errors.New(etxt)
		return
	}

	p = resp.Data

	return
}

// Calls GetPost on the DefaultApplication.
func GetPost(id string) (Post, error) {
	return DefaultApplication.GetPost(id)
}
