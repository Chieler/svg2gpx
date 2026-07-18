"""Write a routed path as a GPX 1.1 track, for Strava / Garmin / Komoot.

Dependency-free: GPX is plain XML, so this writes it directly rather than
pulling in a library. Needs geographic (lat, lon) points -- routes on the
synthetic grid have no real-world location and can't be exported.
"""
from xml.sax.saxutils import escape


def to_gpx(latlon, path, name="svg2gpx route"):
    """Write a sequence of (lat, lon) points as a closed GPX track.

    `latlon` is typically a routed loop's node coordinates already converted
    to WGS84 (see chicago_map.to_wgs84 / the --gpx CLI flag); a closed running
    route repeats its start point at the end, so the written track is a loop
    a watch can follow.
    """
    pts = "\n".join(f'      <trkpt lat="{lat:.6f}" lon="{lon:.6f}"></trkpt>'
                    for lat, lon in latlon)
    xml = ('<?xml version="1.0" encoding="UTF-8"?>\n'
          '<gpx version="1.1" creator="svg2gpx" '
          'xmlns="http://www.topografix.com/GPX/1/1" '
          'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
          'xsi:schemaLocation="http://www.topografix.com/GPX/1/1 '
          'http://www.topografix.com/GPX/1/1/gpx.xsd">\n'
          f'  <trk>\n    <name>{escape(name)}</name>\n    <trkseg>\n{pts}\n'
          '    </trkseg>\n  </trk>\n</gpx>\n')
    with open(path, "w") as f:
        f.write(xml)
    return path
