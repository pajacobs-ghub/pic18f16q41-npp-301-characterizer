// balance_npp301.go
// Given the measures bridge resistances, compute our options for balance resistors.
// Peter J. 2025-03-13

package main

import (
	"fmt"
	"math"
	"os"
	"strconv"
)

// Assume that we can draw resistor values from the E24 series.
var Rvalues []float64 = []float64{ 
	1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0,
	3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1,
	10.0, 11.0, 12.0, 13.0, 15.0, 16.0, 18.0, 20.0, 22.0, 24.0, 27.0, 30.0,
	33.0, 36.0, 39.0, 43.0, 47.0, 51.0, 56.0, 62.0, 68.0, 75.0, 82.0, 91.0,
	100.0, 110.0, 120.0, 130.0, 150.0, 160.0, 180.0, 200.0, 220.0, 240.0, 270.0, 300.0,
	330.0, 360.0, 390.0, 430.0, 470.0, 510.0, 560.0, 620.0, 680.0, 750.0, 820.0, 910.0, 
	1.0e3, 1.1e3, 1.2e3, 1.3e3, 1.5e3, 1.6e3, 1.8e3, 2.0e3, 2.2e3, 2.4e3, 2.7e3, 3.0e3,
	3.3e3, 3.6e3, 3.9e3, 4.3e3, 4.7e3, 5.1e3, 5.6e3, 6.2e3, 6.8e3, 7.5e3, 8.2e3, 9.1e3,
	10.0e3, 11.0e3, 12.0e3, 13.0e3, 15.0e3, 16.0e3, 18.0e3, 20.0e3, 22.0e3, 24.0e3, 27.0e3, 30.0e3,
	33.0e3, 36.0e3, 39.0e3, 43.0e3, 47.0e3, 51.0e3, 56.0e3, 62.0e3, 68.0e3, 75.0e3, 82.0e3, 91.0e3,
}

type NPP301 struct {
	R1, R2, R3, R4 float64
	RA, RB, RC, RD float64
	v2mv6 float64
}

func parallelR(Ra, Rb float64) float64 {
	var Rab float64
	if Ra == 0.0 || Rb == 0.0 {
		Rab = 0.0
	} else {
		Rab = 1.0/(1.0/Ra + 1.0/Rb)
	}
	return Rab
}

func (bridge *NPP301) computeUnbalance() {
	// Balance resistors are in parallel pairs.
	RAB := parallelR(bridge.RA, bridge.RB)
	RCD := parallelR(bridge.RC, bridge.RD)
	// Compute currents in each arm of the bridge.
	i12 := 1.0 / (bridge.R1 + bridge.R2 + RAB)
	i34 := 1.0 / (bridge.R3 + bridge.R4 + RCD)
	// Compute voltages at pins 2 and 6.
	// These are the output pins for the NPP-301.
	v2 := 1.0 - bridge.R1 * i12
	v6 := 1.0 - bridge.R3 * i34
	bridge.v2mv6 = v2 - v6
	return
}

func main() {
	if len(os.Args) != 6 {
		fmt.Println("Expected command-line arguments for R1, R2, R3, R4 and unbalanceTol")
		os.Exit(1)
	}
	// Set the measured resistance values from command-line parameters.
	R1, _ := strconv.ParseFloat(os.Args[1], 64)
	R2, _ := strconv.ParseFloat(os.Args[2], 64)
	R3, _ := strconv.ParseFloat(os.Args[3], 64)
	R4, _ := strconv.ParseFloat(os.Args[4], 64)
	npp := NPP301{R1: R1, R2: R2, R3: R3, R4: R4}
	unbalanceTol, _ := strconv.ParseFloat(os.Args[5], 64)
	fmt.Printf("npp= %v unbalanceTol=%v\n", npp, unbalanceTol)
	// fmt.Printf("Rvalues= %v\n", Rvalues)
	// The initial unbalance is just v2-v6 with zero-value resistors applied.
	npp.computeUnbalance()
	unbalance := npp.v2mv6
	fmt.Printf("initial unbalance v2-v6= %v\n", unbalance)
	var candidates []NPP301
	if unbalance > 0.0 {
		// We set RA=RB=0.0 and check our options for the RC and RD
		for _, RC := range Rvalues {
			for _, RD := range Rvalues {
				nppTest := npp
				nppTest.RA = 0.0
				nppTest.RB = 0.0
				nppTest.RC = RC
				nppTest.RD = RD
				nppTest.computeUnbalance()
				if math.Abs(nppTest.v2mv6) < unbalanceTol {
					candidates = append(candidates, nppTest)
				} 
			}
		}
	} else {
		// We set RC=RD=0.0 and check our options for the RA and RB
		for _, RA := range Rvalues {
			for _, RB := range Rvalues {
				nppTest := npp
				nppTest.RA = RA
				nppTest.RB = RB
				nppTest.RC = 0.0
				nppTest.RD = 0.0
				nppTest.computeUnbalance()
				if math.Abs(nppTest.v2mv6) < unbalanceTol {
					candidates = append(candidates, nppTest)
				} 
			}
		}
	}
	if len(candidates) == 0 {
		fmt.Println("No candidate solutions made the cut.")
	} else {
		for _, c := range candidates {
			fmt.Printf("RA=%.1f RB=%.1f RC=%.1f RD=%.1f v2mv6=%.1e (RAB=%.1f RCD=%.1f)\n",
			c.RA, c.RB, c.RC, c.RD, c.v2mv6,
				parallelR(c.RA, c.RB), parallelR(c.RC, c.RD))
		}
	}
	fmt.Println("Done.")
}
