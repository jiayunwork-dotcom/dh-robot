// Command dh-robot serves a Denavit-Hartenberg forward-kinematics console
// over HTTP. It exposes JSON APIs (/api/fk and /api/skeleton) and a small web
// UI that draws the link skeleton for an open-chain manipulator described by
// standard DH parameters.
package main

import (
	"flag"
	"log"
)

func main() {
	httpAddr := flag.String("http", ":8080", "HTTP listen address; the server also serves the web UI at /")
	flag.Parse()

	srv := newServer(*httpAddr)
	log.Printf("dh-robot listening on %s", *httpAddr)
	if err := srv.run(); err != nil {
		log.Fatal(err)
	}
}
