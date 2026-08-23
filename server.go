package main

import (
	"encoding/json"
	"net/http"

	"dh-robot/internal/dh"
	"dh-robot/internal/kin"
)

// server wires the HTTP routes for dh-robot.
type server struct {
	addr string
}

func newServer(addr string) *server { return &server{addr: addr} }

// routes builds the mux: the two JSON APIs, the example directory and the
// static web UI.
func (s *server) routes() *http.ServeMux {
	mux := http.NewServeMux()
	mux.HandleFunc("/api/fk", s.handleFK)
	mux.HandleFunc("/api/skeleton", s.handleSkeleton)
	mux.Handle("/example/", http.StripPrefix("/example/", http.FileServer(http.Dir("example"))))
	mux.Handle("/", http.FileServer(http.Dir("web")))
	return mux
}

func (s *server) run() error {
	return http.ListenAndServe(s.addr, s.routes())
}

// handleFK decodes a chain spec and returns the forward-kinematics result.
func (s *server) handleFK(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeError(w, "method not allowed; use POST /api/fk", http.StatusMethodNotAllowed)
		return
	}
	var spec kin.ChainSpec
	if err := json.NewDecoder(r.Body).Decode(&spec); err != nil {
		writeError(w, "invalid JSON body: "+err.Error(), http.StatusBadRequest)
		return
	}
	res, err := kin.Forward(spec)
	if err != nil {
		writeError(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, kin.ToFKResponse(spec, res))
}

// handleSkeleton decodes a chain spec and returns the skeleton polyline.
func (s *server) handleSkeleton(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeError(w, "method not allowed; use POST /api/skeleton", http.StatusMethodNotAllowed)
		return
	}
	var spec kin.ChainSpec
	if err := json.NewDecoder(r.Body).Decode(&spec); err != nil {
		writeError(w, "invalid JSON body: "+err.Error(), http.StatusBadRequest)
		return
	}
	pts, err := kin.Skeleton(spec)
	if err != nil {
		writeError(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, kin.ToSkeletonResponse(pts))
}

func writeError(w http.ResponseWriter, msg string, code int) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(map[string]string{"error": msg})
}

func writeJSON(w http.ResponseWriter, v interface{}) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	_ = json.NewEncoder(w).Encode(v)
}

// ensure dh is referenced so the import is meaningful even if the JSON layer
// is the primary consumer of the math package.
var _ = dh.Identity
