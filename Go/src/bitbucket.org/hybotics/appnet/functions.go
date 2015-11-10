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
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"io/ioutil"
	"net/http"
//	"net/url"
//	"strings"
)

func request (r *Request, name string, args EpArgs) (body io.ReadCloser, err error) {
	var path bytes.Buffer

	err = epTemplates.ExecuteTemplate(&path, name, args)

	if err != nil {
		etxt := fmt.Sprintf("appnet.request.functions/*/%s", err)
		err = errors.New(etxt)
		return
	}

	ep := ApiEndpoints[name]
	url := path.String()
	req, err := http.NewRequest(string(ep.Method), url, r.Body)

	if err != nil {
		etxt := fmt.Sprintf("appnet.request.functions/*/%s", err)
		err = errors.New(etxt)
		return
	}

	req.Header.Set("X-ADN-Migration-Overrides", "response_envelope=1")

	if r.Token != "" {
		req.Header.Set("Authorization", "Bearer "+r.Token)
	}

	if r.BodyType != "" {
		req.Header.Set("Content-Type", r.BodyType)
	}

	resp, err := apiHttpClient.Do(req)

	if err != nil {
		etxt := fmt.Sprintf("appnet.request.functions/*/%s", err)
		err = errors.New(etxt)
		return
	}

	body = resp.Body

	return
}

/*
	Do handles all API requests.

	The Request contains the authentication token and optional body.

	The name comes from ApiEndpoints, with template arguments
		provided in args.

	The response is unpacked into v.

	In the future, you would not call this function directly, but
		instead use this helper function for the specific action.
*/
func Do (req *Request, name string, args EpArgs, v interface{}) (err error) {
	body, err := request(req, name, args)

	if err != nil {
		etxt := fmt.Sprintf("appnet.Do.functions/*/%s", err)
		err = errors.New(etxt)
		return
	}

	defer body.Close()

	resp, err := ioutil.ReadAll(body)

	if err != nil {
		etxt := fmt.Sprintf("appnet.request.functions/*/%s", err)
		err = errors.New(etxt)
		return
	}

//	if name == "retrieve realtime user personalized stream" {
//		fmt.Printf("(appnet.Do 1) resp(%d) = '%s'\n", len(resp), resp)
//	}

	err = json.Unmarshal(resp, v)

	if err != nil {
		etxt := fmt.Sprintf("appnet.request.functions/*/%s", err)
		err = errors.New(etxt)
	}

	return
}
