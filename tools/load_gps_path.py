import argparse
import sys
from datetime import datetime
import matplotlib.pyplot as plt
import contextily as cx

def nmea_to_decimal(value, direction):
    """Convert NMEA coordinate (DDMM.MMMM or DDDMM.MMMM) to decimal degrees."""
    if not value:
        return None
    dot_idx = value.find('.')
    deg_len = dot_idx - 2
    degrees = float(value[:deg_len])
    minutes = float(value[deg_len:])
    decimal = degrees + (minutes / 60.0)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def parse_nmea_time(time_str):
    """Parse NMEA UTC timestamp (HHMMSS or HHMMSS.ss) to a datetime object."""
    if not time_str:
        return None
    try:
        if '.' in time_str:
            return datetime.strptime(time_str, "%H%M%S.%f")
        return datetime.strptime(time_str, "%H%M%S")
    except ValueError:
        return None

def load_segments(filepath, threshold_seconds=5.0, quality=1):
    """Extract segments from $GPGGA messages, splitting on gaps >= threshold_seconds.

    Only points with the specified GPS quality indicator are used.
    Quality values: 1 = GPS fix, 2 = DGPS fix, 4 = RTK fix.
    """
    segments = []
    current_segment = []
    last_time = None

    with open(filepath, 'r', encoding='utf-8') as file:
        for line in file:
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                # $GPGGA: parts[1]=Time, parts[2]=Lat, parts[3]=N/S, parts[4]=Lon,
                #         parts[5]=E/W, parts[6]=Quality
                if len(parts) > 6 and parts[1] and parts[2] and parts[4]:
                    try:
                        # Filter by quality indicator
                        if parts[6] != str(quality):
                            continue

                        timestamp = parse_nmea_time(parts[1])
                        lat = nmea_to_decimal(parts[2], parts[3])
                        lon = nmea_to_decimal(parts[4], parts[5])

                        if timestamp is None or lat is None or lon is None:
                            continue

                        if last_time is not None:
                            delta = (timestamp - last_time).total_seconds()
                            if delta >= threshold_seconds:
                                if current_segment:
                                    segments.append(current_segment)
                                current_segment = []

                        current_segment.append((lat, lon))
                        last_time = timestamp

                    except ValueError:
                        continue

    if current_segment:
        segments.append(current_segment)

    return segments

def main():
    parser = argparse.ArgumentParser(
        description='Plot GPS trajectory from NMEA $GPGGA messages, filtered by quality.'
    )
    parser.add_argument(
        'filepath',
        help='Path to the NMEA file'
    )
    parser.add_argument(
        '--quality',
        type=int,
        default=1,
        choices=[1, 2, 4],
        help='GPS quality indicator to filter points (default: 1). '
             '1 = GPS fix, 2 = DGPS fix, 4 = RTK fix'
    )
    args = parser.parse_args()

    segments = load_segments(args.filepath, threshold_seconds=5.0, quality=args.quality)

    if not segments:
        print(f"Error: No valid $GPGGA coordinates with quality {args.quality} found in the file.")
        sys.exit(1)

    # Aggregate coordinates for spatial bounds
    all_lats = [pt[0] for seg in segments for pt in seg]
    all_lons = [pt[1] for seg in segments for pt in seg]

    fig, ax = plt.subplots(figsize=(10, 8))

    # Color palette cycle for segments
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

    # Plot each segment and add start label
    for idx, seg in enumerate(segments, start=1):
        seg_lats = [pt[0] for pt in seg]
        seg_lons = [pt[1] for pt in seg]
        color = colors[(idx - 1) % len(colors)]

        ax.plot(
            seg_lons,
            seg_lats,
            color=color,
            linewidth=2,
            marker='o',
            markersize=3,
            label=f'{idx}'
        )

        # Label start point of segment
        ax.text(
            seg_lons[0],
            seg_lats[0],
            f' {idx}',
            fontsize=10,
            fontweight='bold',
            color=color,
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor=color)
        )

    # Set spatial boundary with margin
    margin = 0.0003
    ax.set_xlim(min(all_lons) - margin, max(all_lons) + margin)
    ax.set_ylim(min(all_lats) - margin, max(all_lats) + margin)

    # Fetch and add OpenStreetMap tile background
    cx.add_basemap(
        ax,
        crs='EPSG:4326',
        source=cx.providers.OpenStreetMap.Mapnik,
        headers={'User-Agent': 'GPS_Trajectory_Visualizer/1.0'}
    )

    ax.set_title(f"GPS Trajectory Plot (Segmented, Quality={args.quality})")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
