package main

import (
	"fmt"
	"log"

	filter "github.com/milosgajdos/go-estimate"
	"github.com/milosgajdos/go-estimate-examples/fallingball"
	"github.com/milosgajdos/go-estimate/estimate"
	"github.com/milosgajdos/go-estimate/kalman/ekf"
	"github.com/milosgajdos/go-estimate/noise"
	"github.com/milosgajdos/go-estimate/sim"
	"github.com/milosgajdos/matrix"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/plot/vg"
)

func main() {

	// ball is the model of the system we will simulate
	ball := fallingball.InputModel(1.0)

	// number of simulation steps
	steps := 50

	// modelOut measurements i.e. true model output state
	modelOut := mat.NewDense(steps, 2, nil)

	// output measurement i.e. output + error
	measOut := mat.NewDense(steps, 2, nil)

	// measurement noise used to simulate real system
	measCov := mat.NewSymDense(1, []float64{100.0})
	measNoise, err := noise.NewGaussian([]float64{0.0}, measCov)
	if err != nil {
		log.Fatalf("Failed to create measurement noise: %v", err)
	}

	// output corrected by filter
	filterOut := mat.NewDense(steps, 2, nil)

	// initial system state and control input
	var x mat.Vector = mat.NewVecDense(2, []float64{100.0, 0.0})
	var u mat.Vector = mat.NewVecDense(1, []float64{-1.0})

	// initial condition
	stateCov := mat.NewSymDense(2, []float64{0.25, 0, 0, 0.25})
	stateNoise, err := noise.NewGaussian([]float64{0.0, 0.0}, stateCov)
	if err != nil {
		log.Fatalf("Failed to create state noise: %v", err)
	}

	// z stores real system measurement: y+noise
	z := new(mat.VecDense)

	// filter initial estimate
	initX := &mat.VecDense{}
	initX.CloneFromVec(x)
	initX.SetVec(0, initX.AtVec(0)+stateNoise.Sample().At(0, 0))
	var est filter.Estimate
	est, err = estimate.NewBase(initX)
	if err != nil {
		log.Fatalf("Failed to create initial estimate: %v", err)
	}

	// initial condition of EKF
	initCond := sim.NewInitCond(x, stateCov)

	f, err := ekf.New(ball, initCond, nil, measNoise)
	if err != nil {
		log.Fatalf("Failed to create EKF filter: %v", err)
	}

	// measure filter correctness
	filterErr := 0.0

	for i := 0; i < steps; i++ {
		// ground truth propagation
		x, err = ball.Propagate(x, u, nil)
		if err != nil {
			log.Fatalf("Model Propagation error: %v", err)
		}

		fmt.Printf("TRUTH State %d:\n%v\n", i, matrix.Format(x))

		// ground truth observation
		y, err := ball.Observe(x, u, nil)
		if err != nil {
			log.Fatalf("Model Observation error: %v", err)
		}

		fmt.Printf("TRUTH Output %d:\n%v\n", i, matrix.Format(y))

		// store results for plotting
		modelOut.Set(i, 0, float64(i))
		modelOut.Set(i, 1, y.AtVec(0))

		// measurement: z = y+noise
		noise := measNoise.Sample()
		fmt.Println("NOISE:", matrix.Format(noise))
		z.AddVec(y, noise)
		// store results for plotting
		measOut.Set(i, 0, float64(i))
		measOut.Set(i, 1, z.AtVec(0))

		fmt.Printf("Measurement %d:\n%v\n", i, matrix.Format(z))

		// propagate particle filters to the next step
		pred, err := f.Predict(est.Val(), u)
		if err != nil {
			log.Fatalf("Filter Prediction error: %v", err)
		}

		// correct state estimate using measurement z
		est, err = f.Update(pred.Val(), u, z)
		if err != nil {
			log.Fatalf("Filter Udpate error: %v", err)
		}

		// calculate filter state error
		xErr := &mat.Dense{}
		xErr.Sub(x, est.Val())
		pInv := &mat.Dense{}
		pInv.Inverse(est.Cov())
		xerrPinv := &mat.Dense{}
		xerrPinv.Mul(xErr.T(), pInv)
		res := &mat.Dense{}
		res.Mul(xerrPinv, xErr)
		filterErr += res.At(0, 0)

		// get corrected output
		yFilter, err := ball.Observe(est.Val(), u, nil)
		if err != nil {
			log.Fatalf("Model Observation error: %v", err)
		}
		fmt.Printf("FILTER Output %d:\n%v\n", i, matrix.Format(yFilter))

		// store results for plotting
		filterOut.Set(i, 0, float64(i))
		filterOut.Set(i, 1, yFilter.AtVec(0))

		fmt.Printf("CORRECTED State  %d:\n%v\n", i, matrix.Format(est.Val()))
		fmt.Printf("CORRECTED Output %d:\n%v\n", i, matrix.Format(yFilter))
		fmt.Println("----------------")
	}

	fmt.Println("XERR:", filterErr)

	plt, err := sim.New2DPlot(modelOut, measOut, filterOut)
	if err != nil {
		log.Fatalf("Failed to make plot: %v", err)
	}

	name := "system.png"
	// Save the plot to a PNG file.
	if err := plt.Save(10*vg.Inch, 10*vg.Inch, name); err != nil {
		log.Fatalf("Failed to save plot to %s: %v", name, err)
	}
}
