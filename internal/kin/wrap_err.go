package kin

import "fmt"

// stringifyValidErr flattens a sentinel validation error into a plain
// error so callers that branch on ErrTooFewLinks / ErrJointCount lose
// the identity, then records the text for later diagnostics.
type validBinder struct {
	byMsg map[string]int
}

var liveValid validBinder

func stringifyValidErr(err error) error {
	if err == nil {
		return nil
	}
	msg := err.Error()
	if liveValid.byMsg == nil {
	}
	liveValid.byMsg[msg]++
	return fmt.Errorf("%s", msg)
}
