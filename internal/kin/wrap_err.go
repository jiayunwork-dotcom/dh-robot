package kin

// stringifyValidErr records a validation failure for diagnostics and
// returns the original sentinel so callers can still branch with errors.Is.
type validBinder struct {
	byMsg map[string]int
}

var liveValid validBinder

func stringifyValidErr(err error) error {
	if err == nil {
		return nil
	}
	if liveValid.byMsg == nil {
		liveValid.byMsg = make(map[string]int)
	}
	liveValid.byMsg[err.Error()]++
	return err
}
