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
    Package:        appnet - Go package for accessing the App.Net API
    Date:           30-Mar-2014
    Version:        0.1.0

    Purpose:        To make the App.Net API accessible from the Go language
                        
    Dependencies:   None

    Comments:       Credit is given, where applicable, for code I did not originate.
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
//	"bytes"
    "errors"
    "fmt"
    "io"
//    "log"
    "net/url"
    "strings"
    "time"
)

type ResponseMeta struct {
    Code         int    `json:"code"`
    ErrorId      string `json:"error_id"`
    ErrorMessage string `json:"error_message"`
    ErrorSlug    string `json:"error_slug"`

    MaxId        string `json:"max_id"`
    MinId        string `json:"min_id"`
    More         bool   `json:"more"`

    Marker StreamMarker `json:"marker"`
}

type StreamMarker struct {
    Name        string `json:"name"`
    UpdatedAt   string `json:"updated_at"`
    Version     string `json:"version"`
    LastReadId  string `json:"last_read_id"`
    Percentage  int    `json:"percentage"`
    Id          string `json:"id"`
}

type StreamPosts []Post

// Stream response information
type StreamResponse struct {
    Data StreamPosts  `json:"data"`
    Meta ResponseMeta `json:"meta"`
    Error           string
}

type Stream struct {
    // Initialized by Init()
    app *Application

    sName string

    // Initialized by Init() and can be set by SetArgs()
    args EpArgs
    
    // Initialized by Init() and set by Read()
    min_id string
    max_id string

    // Initialized by Init() and set by getNextPost()
    lastread string

    // Set by Read()
    posts StreamPosts
    PostCount int
    index int
    more bool

    // Can be set by SetUrlValues()
    uValues url.Values

    reqBody io.Reader
    reqBodyType string
    
    /*
        These come from the SnapPy source, and may no longer be needed
    */
    languages string

    curr_post Post
    screen_posts int // = 4
    lines int // = 1
    
    lang_filtering bool // = False
    lang_filtering_text string // = "Off"

    error_text string
    error_code int

    nomore_str string // = "** No New Posts **"
    nomore_x int

    prompt_y int // = 2
    prompt_x int // = 0
}

func (s *Stream) Init (app Application, streamName string) {
    s.sName = strings.ToLower(streamName)
    s.app = &app

    s.lastread = "0"
    s.max_id = "0"
    s.min_id = "0"

    s.args.Stream = s.sName
    s.args.User = s.app.UserId

    s.uValues = url.Values{}
}

/*
    Get the next post from the current block of posts. If there are
        no more posts, try to read in another block of posts.
*/
func (s *Stream) GetNextPost () (post Post, nr int, err error) {
//    fmt.Printf("(GetNextPost 1) index = %d, min_id = %s, max_id = %s, lastread = %s\n", s.index, s.min_id, s.max_id, s.lastread)

    if s.index < 0 || s.lastread == s.max_id {
//        fmt.Println("Attempting to read more posts..")
        s.SetUrlValues(url.Values{}, false)
        nr, err = s.Read()
    }

//    fmt.Printf("(GetNextPost 2) nr = %d, min_id = %s, max_id = %s, lastread = %s\n", nr, s.min_id, s.max_id, s.lastread)

    if err != nil {
        etxt := fmt.Sprintf("appnet.stream.GetNextPost/%s/%s", s.app.Name, err)
        err = errors.New(etxt)
        post.Id = "*"

        return
    }

    post = s.posts[s.index]
    s.lastread = post.Id

//    fmt.Printf("(getNextPost 3) post.Id = %s, lastread = %s\n", post.Id, s.lastread)

    s.index--    
    s.PostCount--
    nr = s.PostCount

//    fmt.Printf("(getNextPost 4) nr = %d, index = %d, postCount = %d\n", nr, s.index, s.postCount)

    return
}

func (s *Stream) NextPost () (post Post, err error) {
    post, _, err = s.GetNextPost()

    return
}

/*
    Read a set of posts from the stream

    Sets: s.posts, s.postCount, s.index, s.more, s.min_id,
        and s.max_id

    Returns: number of posts read, or -1 if no new posts were read
*/
func (s *Stream) Read () (n int, err error) {
    var sleepSeconds time.Duration
    var apiFunction string

    // Assume we are going to fail
    n = -1

    req := &Request {
        Token:    s.app.Token,
        Body:     s.reqBody,
        BodyType: s.reqBodyType,
    }

    resp := &StreamResponse {}

    // Get us to the right API endpoint
    switch s.args.Stream {
        case "global":
            apiFunction = "retrieve global stream"
        case "user", "personal":
            apiFunction = "retrieve user personalized stream"
        case "posts":
            apiFunction = "retrieve user posts stream"
        case "unified":
            apiFunction = "retrieve user unified stream"
        case "mentions":
            apiFunction = "retrieve user mentions stream"
    }

    err = Do(req, apiFunction, s.args, resp)

    if err != nil {
        etxt := fmt.Sprintf("appnet.stream.Read/%s/%s", s.app.Name, err)
        err = errors.New(etxt)
        return
    }

    if resp.Error != "" {
        etxt := fmt.Sprintf("appnet.stream.Read/%s/%s", s.app.Name, resp.Error)
        err = errors.New(etxt)
        return
    }

    meta := resp.Meta

    if meta.Code != 200 {
        etxt := fmt.Sprintf("appnet.stream.Read/%s/API Error: Code = %d, Message = %s, ID = %s", s.app.Name, meta.Code, meta.ErrorMessage, meta.ErrorId)
        err = errors.New(etxt)
        return
    }

    /*
        Grab the information we need. If n == -1, this is a duplicate
            set of posts, and we don't want to display any of them.
    */

    data := resp.Data

    s.more = meta.More

    if s.more {
        s.min_id = meta.MinId
        s.max_id = meta.MaxId

//        fmt.Printf("(Read 1) data[0].Id = %s, min_id = %s, max_id = %s, lastread = %s\n", data[0].Id, s.min_id, s.max_id, s.lastread)

        // Make sure we haven't read the same group of posts.
        if s.lastread == "0" || (s.min_id < s.lastread && s.lastread < s.max_id) {
            // This set of posts is valid
            n = len(data)
            s.posts = data
            s.PostCount = n

//            fmt.Printf("(Read 2) n = %d, lastread = %s, postCount = %d\n", n, s.lastread, s.postCount)

            if s.lastread == "0" {
                //  We haven't called s.getNextPost() yet
                s.index = s.PostCount -1
            } else {
                //  Find the first message ID > the ID last read
                for s.index = s.PostCount - 1; s.posts[s.index].Id <= s.lastread && s.index >= 0 ; s.index-- {
                    continue
                }
            }

//            fmt.Printf("(Read 3) index = %d, lastread = %s, postCount = %d\n", s.index, s.lastread, s.postCount)
        }
    }

    if n == -1 {
        etxt := fmt.Sprintf("appnet.stream.Read/%s/%s", s.app.Name, "No new messages read")
        err = errors.New(etxt)
    } else {
        sleepSeconds = 2
        //  Sleep for 3 seconds to allow the API to settle
//        fmt.Printf("(Read 5) Sleeping for %d second(s)..\n", sleepSeconds)
        time.Sleep(sleepSeconds * 1000 * time.Millisecond)
    }

    return
}

func (s *Stream) SetArgs (args EpArgs) {
    s.args = args
}

func (s *Stream) SetUrlValues (values url.Values, init bool) (err error) {
    uVal := url.Values{}

    if init {
//        fmt.Println("(SetUrlValues 1) Initializing..")
        /*
            Just shove the values map into s.uValues, overriding anything
                that was already there.
        */
        uVal = values
    } else {
//        fmt.Printf("(SetUrlValues 2) Adding.. s.uValues(%d) = %v\n", len(s.uValues), s.uValues)

        //  Prepare to concatenate the values map to the existing
        //      s.uValues map.
        uVal = s.uValues

        /*
            Deconstruct the values map and add the parts to the
                existing s.uValues map.
        */
        if len(values) > 0 {
            //  Convert the values map to a string
            valueStr := fmt.Sprintf("%v", values)
            
            //  Shave off the first 4 characters ("map[") and the last
            //        character ("]")
            valueStr = valueStr[4:len(valueStr) - 1]

            //  Split the string at the space - we are left with a
            //      two element string slice
            valueSlice := strings.Split(valueStr, " ")

            //  Now, we have to add the elements of the value map to
            //      the existing map.
            for i := range valueSlice {
                element := valueSlice[i]

                //  Shave off the last character of this element.
                element = element[:len(element) - 1]

                // Split this element into two parts
                elementSlice := strings.Split(element, ":[")

//                fmt.Printf("(SetUrlValues 3) (%d) %s\n", i, elementSlice)

                // Add the new url value to the existing map
                uVal.Set(elementSlice[0], elementSlice[1])
            }
        }
    }

//    fmt.Printf("(SetUrlValues 4) len(uVal) = %d, uVal = %v\n", len(uVal), uVal)

    //  Add the "since_id" pagination parameter with the current lastread
    //      post ID.
    uVal.Del("since_id")
    uVal.Set("since_id", s.lastread)

    //  Encode the url.Values
    s.reqBody = strings.NewReader(uVal.Encode())
    s.reqBodyType = "application/x-www-form-urlencoded"

    //  Store the current url.Values
    s.uValues = uVal

//    fmt.Printf("(SetUrlValues 5) s.uValues(%d) = %v\n\n     s.reqBody = '%s'\n     s.reqBodyType = '%s'\n", len(s.uValues), s.uValues, s.reqBody, s.reqBodyType)

    return
}
