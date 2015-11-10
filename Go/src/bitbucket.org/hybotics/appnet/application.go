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
	"bytes"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strings"
)

type Application struct {
	Id				string
	Name			string
	Date			string
	Version			string
	Windowed		bool
	Secret			string
	RedirectURI		string
	Scopes			Scopes
	PasswordSecret	string
	Token			string // Access token, Added, DAW, 06-Nov-2013
	UserName		string // User name, Added, DAW, 06-Nov-2013
	UserId			string // User ID, Added, DAW, 06-Nov-2013
}

var DefaultApplication = &Application{}
var apiHttpClient = &http.Client{}

type Request struct {
	Token    string    // Authentication token for the user or ""
	Body     io.Reader // Data for the body
	BodyType string    // Value for the Content-Type header
}

// Generate the authentication URL for the server-side flow.
func (c *Application) AuthenticationURL(state string) (string, error) {
	var url bytes.Buffer

	args := struct {
		*Application
		State string
	}{c, state}

	err := epTemplates.ExecuteTemplate(&url, "authentication url", args)

	if err != nil {
		return "", err
	}

	return url.String(), nil
}

/*
	During server-side flow, the user will be redirected back with a
		code.

	AccessToken() uses this code to request an access token for the
		user, which is returned as a string.
*/
func (c *Application) AccessToken(code string) (string, error) {
	data := url.Values{}
	data.Set("client_id", c.Id)
	data.Set("client_secret", c.Secret)
	data.Set("grant_type", "authorization_code")
	data.Set("redirect_uri", c.RedirectURI)
	data.Set("code", code)

	r := &Request{
		Body:     strings.NewReader(data.Encode()),
		BodyType: "application/x-www-form-urlencoded",
	}

	resp := &struct {
		AccessToken string `json:"access_token"`
		Error       string
	}{}

	err := Do(r, "get access token", EpArgs{}, resp)

	if err != nil {
		return "", err
	}

	if resp.Error != "" {
		return "", errors.New(resp.Error)
	}

	return resp.AccessToken, nil
}

/*
	PasswordToken is used to carry out the password flow. The function
		submits the username and password to get an access token. This
		token is returned as a string.

		** Works **
*/
func (c *Application) PasswordToken (userName, userPassword string) (aToken string, err error) {
	type Response struct {
		AccessToken string `json:"access_token"`
		Error       string
	}

	data := url.Values{}
	data.Set("client_id", c.Id)
	data.Set("password_grant_secret", c.PasswordSecret)
	data.Set("grant_type", "password")
	data.Set("username", userName)
	data.Set("password", userPassword)
	data.Set("scope", c.Scopes.Spaced())

	r := &Request{
		Body:     strings.NewReader(data.Encode()),
		BodyType: "application/x-www-form-urlencoded",
	}

	resp := &Response{}

	err = Do(r, "get access token", EpArgs{}, resp)

	aToken = ""

	if err != nil {
		return
	}

	if resp.Error != "" {
		err = errors.New(resp.Error)
	} else {
		aToken = resp.AccessToken
	}

	return
}

/*
    Authenticate a user via the Password Flow method

    Returns: Access Token and User ID in app
*/
func (c *Application) PasswordFlowAuthenticator (userName, userPassword string, appScope Scopes) (aToken string, user User, err error) {
    // Set the scope for this authentication
    c.Scopes = appScope

    // Try authenticating with the user information we have
    aToken, err = c.PasswordToken(userName, userPassword)

    if err != nil {
        etxt := fmt.Sprintf("appnet.application.PasswordFlowAuthenticator/*/%s", err)
        err = errors.New(etxt)

        return
    }

    // We need to get the user ID from the user name
    user, err = c.GetUser("@" + userName)

    if err != nil {
        etxt := fmt.Sprintf("appnet.application.PasswordFlowAuthenticator/*/%s", err)
        err = errors.New(etxt)

        return
    }

    return
}