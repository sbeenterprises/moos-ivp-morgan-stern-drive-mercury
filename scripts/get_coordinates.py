#!/usr/bin/env python3

import rasterio
from rasterio.crs import CRS
from rasterio.warp import transform_bounds

def get_geotiff_info(geotiff_path):
    """
    Opens a GeoTIFF and returns:
      lat_north, lat_south, lon_east, lon_west, datum_lat, datum_lon
    in WGS84 (EPSG:4326).
    """
    with rasterio.open(geotiff_path) as src:
        # Original bounding box in the source CRS
        left, bottom, right, top = src.bounds
        src_crs = src.crs

        # We want to ensure the bounding box is in EPSG:4326
        dst_crs = CRS.from_epsg(4326)

        # Transform bounds to EPSG:4326 if necessary
        west, south, east, north = transform_bounds(
            src_crs,        # Source CRS
            dst_crs,        # Destination CRS (WGS84)
            left, bottom, right, top
        )

        # Calculate the center (datum) latitude/longitude
        datum_lat = (north + south) / 2.0
        datum_lon = (east + west) / 2.0

        # Return the values in a dict for convenience
        return {
            "lat_north": north,
            "lat_south": south,
            "lon_east":  east,
            "lon_west":  west,
            "datum_lat": datum_lat,
            "datum_lon": datum_lon
        }

def main():
    geotiff_path = "morgan_city.tif"  # Replace with your GeoTIFF file path
    info = get_geotiff_info(geotiff_path)

    print("GeoTIFF Info:")
    print(f"  lat_north = {info['lat_north']}")
    print(f"  lat_south = {info['lat_south']}")
    print(f"  lon_east  = {info['lon_east']}")
    print(f"  lon_west  = {info['lon_west']}")
    print(f"  datum_lat = {info['datum_lat']}")
    print(f"  datum_lon = {info['datum_lon']}")

if __name__ == "__main__":
    main()
