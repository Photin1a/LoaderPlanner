#!/bin/bash

gnuplot -e "plot "v.txt" with lines title 'v'"
gnuplot -e "plot "gamma.txt" with lines title 'gamma'"
gnuplot -e "plot "theta.txt" with lines title 'theta'"
gnuplot -e "plot "a.txt" with lines title 'a'"
gnuplot -e "plot "omega.txt" with lines title 'omega'"
gnuplot -e "plot "jerk.txt" with lines title 'jerk'"


