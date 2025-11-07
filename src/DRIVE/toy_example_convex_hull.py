#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import csv
from shapely.geometry import Polygon, MultiPoint
from shapely import concave_hull  # Shapely >=2.0
import numpy as np

points = []
plot_lines = []
current_hull = []  # store currently drawn hull elements
concave_ratio = 0.3
max_distance = 0.5    # max distance between consecutive points


def interpolate_points(p1, p2, max_dist=0.1):
    """Return a list of points between p1 and p2 so that distance <= max_dist."""
    line = np.array([p1, p2])
    dist = np.linalg.norm(line[1] - line[0])
    if dist <= max_dist:
        return [tuple(p2)]
    n = int(np.ceil(dist / max_dist))
    xs = np.linspace(p1[0], p2[0], n + 1)[1:]  # skip first point
    ys = np.linspace(p1[1], p2[1], n + 1)[1:]
    return list(zip(xs, ys))


def onclick(event):
    """Add interpolated points along the segment to last point."""
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    new_point = (event.xdata, event.ydata)
    if points:
        last_point = points[-1]
        new_points = interpolate_points(np.array(last_point), np.array(new_point), max_distance)
        for pt in new_points:
            points.append(pt)
            (p,) = ax.plot(pt[0], pt[1], "bo")
            plot_lines.append(p)
            if len(points) > 1:
                x_prev, y_prev = points[-2]
                (l,) = ax.plot([x_prev, pt[0]], [y_prev, pt[1]], "b--")
                plot_lines.append(l)
    else:
        points.append(new_point)
        (p,) = ax.plot(new_point[0], new_point[1], "bo")
        plot_lines.append(p)

    # Close polygon if >=3 points
    if len(points) >= 3:
        if hasattr(onclick, "closing_line") and onclick.closing_line:
            onclick.closing_line.remove()
        x_first, y_first = points[0]
        x_last, y_last = points[-1]
        (l,) = ax.plot([x_last, x_first], [y_last, y_first], "b--")
        onclick.closing_line = l
        plot_lines.append(l)

    plt.draw()


onclick.closing_line = None


def save_to_csv(event):
    if not points:
        print("⚠️ No points to save.")
        return
    filename = "polygon_points.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(points)
    print(f"✅ Points saved to {filename}")


def on_key(event):
    if event.key == "enter" and len(points) >= 3:
        polygon = Polygon(points)
        xs, ys = polygon.exterior.xy
        poly_patch = ax.fill(xs, ys, color="skyblue", alpha=0.3)
        plot_lines.extend(poly_patch)
        plt.draw()
        print("✅ Polygon displayed.")


# --- Hull Buttons ---
def create_convex_hull(event):
    global current_hull
    # remove previous hulls
    for item in current_hull:
        item.remove()
    current_hull.clear()

    if len(points) < 3:
        print("⚠️ Need at least 3 points for convex hull.")
        return
    hull = MultiPoint(points).convex_hull
    xs, ys = hull.exterior.xy
    (l,) = ax.plot(xs, ys, color="red", linewidth=2)
    f = ax.fill(xs, ys, color="red", alpha=0.2)
    current_hull.extend([l] + list(f))
    plt.draw()
    print("✅ Convex hull drawn.")


def create_concave_hull(event):
    global current_hull
    # remove previous hulls
    for item in current_hull:
        item.remove()
    current_hull.clear()
    

    if len(points) < 4:
        print("⚠️ Need at least 4 points for concave hull.")
        return
    try:
        hull = concave_hull(MultiPoint(points), ratio=concave_ratio)
        print(hull.exterior.coords)
        if hull.is_empty or hull.geom_type != "Polygon":
            print("❌ Concave hull could not be computed.")
            return
        xs, ys = hull.exterior.xy
        (l,) = ax.plot(xs, ys, color="green", linewidth=2)
        f = ax.fill(xs, ys, color="green", alpha=0.2)
        current_hull.extend([l] + list(f))
        plt.draw()

        
        print(hull.exterior.coords[0])
        print(f"✅ Concave hull drawn (ratio={concave_ratio:.2f})")
    except Exception as e:
        print(f"Error computing concave hull: {e}")


def update_ratio(val):
    """Update slider value, doesn't recompute hull automatically."""
    global concave_ratio
    concave_ratio = val


def reset_plot(event):
    """Clear points, lines, and hulls."""
    points.clear()
    # Remove all lines and points
    for item in plot_lines:
        try:
            item.remove()
        except Exception:
            pass
    plot_lines.clear()

    # Remove any drawn hulls
    for item in current_hull:
        try:
            item.remove()
        except Exception:
            pass
    current_hull.clear()

    # Remove closing line if it exists
    if hasattr(onclick, "closing_line") and onclick.closing_line:
        try:
            onclick.closing_line.remove()
        except Exception:
            pass
        onclick.closing_line = None

    plt.draw()
    print("🔄 Plot reset.")


def main():
    global ax, ratio_slider
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.35)
    ax.set_title("Polygon with Interpolated Points")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)

    fig.canvas.mpl_connect("button_press_event", onclick)
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Buttons
    save_ax = plt.axes([0.46, 0.05, 0.10, 0.075])
    convex_ax = plt.axes([0.58, 0.05, 0.10, 0.075])
    concave_ax = plt.axes([0.70, 0.05, 0.12, 0.075])
    reset_ax = plt.axes([0.84, 0.05, 0.10, 0.075])

    save_button = Button(save_ax, "Save CSV")
    convex_button = Button(convex_ax, "Convex Hull")
    concave_button = Button(concave_ax, "Concave Hull")
    reset_button = Button(reset_ax, "Reset")

    save_button.on_clicked(save_to_csv)
    convex_button.on_clicked(create_convex_hull)
    concave_button.on_clicked(create_concave_hull)
    reset_button.on_clicked(reset_plot)

    # Slider (affects concave ratio but not automatically redraw)
    slider_ax = plt.axes([0.15, 0.12, 0.25, 0.03], facecolor="lightgray")
    ratio_slider = Slider(slider_ax, "Concave Ratio", valmin=0.05, valmax=1.0, valinit=concave_ratio, valstep=0.05)
    ratio_slider.on_changed(update_ratio)

    plt.show()


if __name__ == "__main__":
    main()
