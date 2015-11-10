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
	"fmt"
	"text/template"
)

const (
	apiHost     = "https://alpha-api.app.net"
	apiAuthHost = "https://alpha.app.net"
)

type httpMethod string

const (
	httpGet    httpMethod = "GET"
	httpPost              = "POST"
	httpDelete            = "DELETE"
)

type endpoint struct {
	Method  httpMethod
	Path    string
	Options *epOptions
}

var ApiEndpoints = map[string]endpoint{
	// Users
	"retrieve user":        {httpGet, apiHost + "/stream/0/users/{{.User}}", nil},
	"follow user":          {httpPost, apiHost + "/stream/0/users/{{.User}}/follow", nil},
	"unfollow user":        {httpDelete, apiHost + "/stream/0/users/{{.User}}/follow", nil},
	"list followed users":  {httpGet, apiHost + "/stream/0/users/{{.User}}/following", nil},
	"list following users": {httpGet, apiHost + "/stream/0/users/{{.User}}/followers", nil},
	"mute user":            {httpPost, apiHost + "/stream/0/users/{{.User}}/mute", nil},
	"unmute user":          {httpDelete, apiHost + "/stream/0/users/{{.User}}/mute", nil},
	"list muted users":     {httpGet, apiHost + "/stream/0/users/me/muted", nil},
	"search for users":     {httpGet, apiHost + "/stream/0/users/search", nil},
	"list reposters":       {httpGet, apiHost + "/stream/0/posts/{{.Post}}/reposters", nil},
	"list starrers":        {httpGet, apiHost + "/stream/0/posts/{{.Post}}/star", nil},

	// Tokens
	"check current token": {httpGet, apiHost + "/stream/0/token", nil},

	// Posts
	"create post":                       {httpPost, apiHost + "/stream/0/posts", nil},
	"retrieve post":                     {httpGet, apiHost + "/stream/0/posts/{{.Post}}", nil},
	"delete post":                       {httpDelete, apiHost + "/stream/0/posts/{{.Post}}", nil},
	"retrieve post replies":             {httpGet, apiHost + "/stream/0/posts/{{.Post}}/replies", nil},
	"retrieve user posts":               {httpGet, apiHost + "/stream/0/users/{{.User}}/posts", nil},
	"repost post":                       {httpPost, apiHost + "/stream/0/posts/{{.Post}}/repost", nil},
	"unrepost post":                     {httpDelete, apiHost + "/stream/0/posts/{{.Post}}/repost", nil},
	"star post":                         {httpPost, apiHost + "/stream/0/posts/{{.Post}}/star", nil},
	"unstar post":                       {httpDelete, apiHost + "/stream/0/posts/{{.Post}}/star", nil},
	"retrieve user starred posts":       {httpGet, apiHost + "/stream/0/users/{{.User}}/stars", nil},
	"retrieve tagged posts":             {httpGet, apiHost + "/stream/0/posts/tag/{{.Hashtag}}", nil},

	// Streams
	"retrieve global stream":						{httpGet, apiHost + "/stream/0/posts/stream/global", nil},
	"retrieve public stream":						{httpGet, apiHost + "/stream/0/streams/public", nil},
	"retrieve multiple user personalized stream":	{httpGet, apiHost + "/stream/0/streams/app", nil},
	"retrieve user personalized stream":			{httpGet, apiHost + "/stream/0/posts/stream", nil},
	"retrieve user posts stream":					{httpGet, apiHost + "/stream/0/users/{{.User}}/posts", nil},
	"retrieve user unified stream":					{httpGet, apiHost + "/stream/0/posts/stream/unified", nil}, 
	"retrieve user mentions stream":				{httpGet, apiHost + "/stream/0/users/{{.User}}/mentions", nil}, 
	"retrieve stream status":						{httpGet, apiHost + "/stream/0/streams/{{.Stream}}", nil},
	"control stream":								{httpPost, apiHost + "/stream/0/streams/{{.Stream}}", nil},

	// Real-time updates
	"list subscriptions":       {httpGet, apiHost + "/stream/0/subscriptions", nil},
	"create subscription":      {httpPost, apiHost + "/stream/0/subscriptions", nil},
	"delete subscription":      {httpDelete, apiHost + "/stream/0/subscriptions/{{.Subscription}}", nil},
	"delete all subscriptions": {httpDelete, apiHost + "/stream/0/subscriptions", nil},

	// Filters
	"retrieve current user filters": {httpGet, apiHost + "/stream/0/filters", nil},
	"create filter":                 {httpPost, apiHost + "/stream/0/filters", nil},
	"retrieve filter":               {httpGet, apiHost + "/stream/0/filters/{{.Filter}}", nil},
	"delete filter":                 {httpGet, apiHost + "/stream/0/filters/{{.Filter}}", nil},

	// Authentication (Server-side flow)
	"authentication url": {httpGet, apiAuthHost + "/oauth/authenticate?client_id={{urlquery .Id}}&response_type=code&redirect_uri={{urlquery .RedirectURI}}&scope={{urlquery .Scopes.Spaced}}{{if .State}}&state={{urlquery .State}}{{end}}", nil},
	"get access token":   {httpPost, apiAuthHost + "/oauth/access_token", &epOptions{ResponseEnvelope: false}},
}

// These fields are used to fill the endpoint template.
type EpArgs struct {
	User, Post, Hashtag, Stream, Subscription, Filter string
}

type epOptions struct {
	ResponseEnvelope bool // Do we expect a response envelope?
}

var epTemplates = new(template.Template)

func init() {
	for k, v := range ApiEndpoints {
		template.Must(epTemplates.New(k).Parse(v.Path))
	}
}

type responseEnvelope struct {
	Data interface{}  `json:"data"`
	Meta responseMeta `json:"meta"`
}

type responseMeta struct {
	Code         int    `json:"code"`
	ErrorId      string `json:"error_id"`
	ErrorMessage string `json:"error_message"`
}

// Represents errors returned by ADN as a result of an API call.
type APIError responseMeta

func (e APIError) Error() string {
	return fmt.Sprintf("%d: %s (%s)", e.Code, e.ErrorMessage, e.ErrorId)
}
