package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"

	"github.com/shanghuiyang/kalman"
)

type FeatureCollection struct {
	Type     string    `json:"type"`
	Features []Feature `json:"features"`
}

type Feature struct {
	Type       string     `json:"type"`
	Properties Properties `json:"properties"`
	Geometry   Geometry   `json:"geometry"`
}

type Properties struct {
	Type string `json:"type"`
}

type Geometry struct {
	Coordinates [][]float64 `json:"coordinates"`
	Type        string      `json:"type"`
}

func loadLineString(path string) ([]kalman.Point, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var fc FeatureCollection
	if err := json.Unmarshal(raw, &fc); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	if len(fc.Features) == 0 {
		return nil, fmt.Errorf("%s: no features found", path)
	}
	coords := fc.Features[0].Geometry.Coordinates
	pts := make([]kalman.Point, len(coords))
	for i, c := range coords {
		pts[i] = kalman.Point{Lon: c[0], Lat: c[1]}
	}
	return pts, nil
}

/*
usage:
./kalman -r route.geojson -g gps.geojson -o estimated.geojson
*/
func main() {
	var routeFile, gpsFile, outputFile string
	flag.StringVar(&routeFile, "r", "", "path to planned route geojson (predicted route)")
	flag.StringVar(&routeFile, "route", "", "path to planned route geojson (measured route)")
	flag.StringVar(&gpsFile, "g", "", "path to GPS geojson (measured route)")
	flag.StringVar(&gpsFile, "gps", "", "path to GPS geojson (measured route)")
	flag.StringVar(&outputFile, "o", "output.geojson", "output path for estimated route")
	flag.StringVar(&outputFile, "output", "output.geojson", "output path for estimated route")
	flag.Parse()

	if routeFile == "" || gpsFile == "" {
		fmt.Fprintln(os.Stderr, "Usage: kalman-filtering -r <route.geojson> -g <gps.geojson> [-o <output.geojson>]")
		flag.PrintDefaults()
		os.Exit(1)
	}

	route, err := loadLineString(routeFile)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	gps, err := loadLineString(gpsFile)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}

	f := kalman.NewFilter(route, kalman.DefaultConfig())
	estimated := make([]kalman.Point, len(gps))
	for i, p := range gps {
		estimated[i] = f.Update(p)
	}

	coords := make([][]float64, len(estimated))
	for i, p := range estimated {
		coords[i] = []float64{p.Lon, p.Lat}
	}

	out := FeatureCollection{
		Type: "FeatureCollection",
		Features: []Feature{{
			Type:       "Feature",
			Properties: Properties{Type: "estimated"},
			Geometry: Geometry{
				Coordinates: coords,
				Type:        "LineString",
			},
		}},
	}

	buf, _ := json.MarshalIndent(out, "", "  ")
	if err := os.WriteFile(outputFile, buf, 0644); err != nil {
		fmt.Fprintf(os.Stderr, "write %s: %v\n", outputFile, err)
		os.Exit(1)
	}
	fmt.Printf("saved %d estimated points: %s\n", len(estimated), outputFile)
}
