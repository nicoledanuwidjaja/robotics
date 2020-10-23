// Cairo Drawing Window
// Based on Gnome doc example.
// ref: https://developer.gnome.org/gtk3/stable/ch01s05.html

#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
using namespace std;

typedef lock_guard<mutex> guard;

#include <X11/Xlib.h>
#include <gtk/gtk.h>
#include "viz.hh"

/* Surface to store current scribbles */
std::mutex mx;
static cairo_surface_t* surface = NULL;
static GtkWidget *window = NULL;
static GtkWidget *drawing_area = NULL;
GMutex mutex_interface;

/* Map properties */
int START_X = -20;
int START_Y = -20;
int W_WIDTH, W_HEIGHT; // default: 618 x 618


static void
clear_surface (void)
{
    cairo_t *cr;

    cr = cairo_create(surface);

    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_paint(cr);

    cairo_destroy(cr);
}

/* Create a new surface of the appropriate size to store our scribbles */
static gboolean
configure_event_cb(GtkWidget         *widget,
                   GdkEventConfigure *event,
                   gpointer           data)
{
    g_mutex_lock(&mutex_interface);
    if (surface) {
        cairo_surface_destroy(surface);
    }

    surface = gdk_window_create_similar_surface(
        gtk_widget_get_window(widget),
        CAIRO_CONTENT_COLOR,
        gtk_widget_get_allocated_width(widget),
        gtk_widget_get_allocated_height(widget)
    );

    /* Initialize the surface to white */
    clear_surface();

    /* We've handled the configure event, no need for further processing. */
    g_mutex_unlock(&mutex_interface);
    return TRUE;
}

/* Redraw the screen from the surface. Note that the ::draw
 * signal receives a ready-to-be-used cairo_t that is already
 * clipped to only draw the exposed areas of the widget
 */
static gboolean
draw_cb(GtkWidget *widget,
        cairo_t   *cr,
        gpointer   data)
{
    g_mutex_lock(&mutex_interface);
    cairo_set_source_surface(cr, surface, 0, 0);
    cairo_paint(cr);
    g_mutex_unlock(&mutex_interface);
    return FALSE;
}

/* Draw a rectangle on the surface at the given position */
static void
draw_brush(GtkWidget *widget,
           gdouble    x,
           gdouble    y)
{
    GDK_THREADS_ENTER();
    guard _gg(mx);
    cairo_t *cr;

    //GdkRectangle rect;
    //gtk_widget_get_allocation(widget, &rect);
    //cout << "rect: " << rect.x << "," << rect.y << endl;

    /* Paint to the surface, where we store our state */
    cr = cairo_create(surface);

    cout << "Draw rectangle" << endl;
    cairo_rectangle(cr, x - 1, y - 1, 2, 2);
    cairo_fill(cr);

    cairo_destroy(cr);

    /* Now invalidate the affected region of the drawing area. */
    gtk_widget_queue_draw_area(widget, x - 1, y - 1, 2, 2);
    GDK_THREADS_LEAVE();
}

/* Draw a colored rectangle to represent a wall at given position */
static void 
draw_wall(GtkWidget *widget,
           gdouble    x,
           gdouble    y)
{
    GDK_THREADS_ENTER();
    guard _gg(mx);
    cairo_t *cr;

    //GdkRectangle rect;
    //gtk_widget_get_allocation(widget, &rect);
    //cout << "rect: " << rect.x << "," << rect.y << endl;

    /* Paint to the surface, where we store our state */
    cr = cairo_create(surface);

    cout << "Draw occupied space" << endl;
    cairo_rectangle(cr, x - 1, y - 1, 10, 10);
    cairo_set_source_rgb(cr, 200, 28, 0);
    cairo_fill(cr);

    cairo_destroy(cr);

    /* Now invalidate the affected region of the drawing area. */
    gtk_widget_queue_draw_area(widget, x - 1, y - 1, 10, 10);
    GDK_THREADS_LEAVE();
}

/* Draw a colored rectangle to represent a free space at given position */
static void 
draw_free(GtkWidget *widget,
           gdouble    x,
           gdouble    y)
{
    GDK_THREADS_ENTER();
    guard _gg(mx);
    cairo_t *cr;

    //GdkRectangle rect;
    //gtk_widget_get_allocation(widget, &rect);
    //cout << "rect: " << rect.x << "," << rect.y << endl;

    /* Paint to the surface, where we store our state */
    cr = cairo_create(surface);

    cout << "Draw free space" << endl;
    cairo_rectangle(cr, x - 1, y - 1, 3, 3);
    cairo_set_source_rgb(cr, 0.55, 0.55, 0.55);
    cairo_fill(cr);

    cairo_destroy(cr);

    /* Now invalidate the affected region of the drawing area. */
    gtk_widget_queue_draw_area(widget, x - 1, y - 1, 3, 3);
    GDK_THREADS_LEAVE();
}


/* Handle button press events by either drawing a rectangle
 * or clearing the surface, depending on which button was pressed.
 * The ::button-press signal handler receives a GdkEventButton
 * struct which contains this information.
 */
static gboolean
button_press_event_cb(GtkWidget      *widget,
                      GdkEventButton *event,
                      gpointer        data)
{
    cout << "click" << endl;

    g_mutex_lock(&mutex_interface);
    /* paranoia check, in case we haven't gotten a configure event */
    if (surface == NULL)
        return FALSE;

    /*
    if (event->button == GDK_BUTTON_PRIMARY)
    {
        draw_brush(widget, event->x, event->y);
    }
    else 
    */
      
    if (event->button == GDK_BUTTON_SECONDARY)
    {
        clear_surface();
        gtk_widget_queue_draw(widget);
    }

    /* We've handled the event, stop processing */
    g_mutex_unlock(&mutex_interface);
    return TRUE;
}

/* Handle motion events by continuing to draw if button 1 is
 * still held down. The ::motion-notify signal handler receives
 * a GdkEventMotion struct which contains this information.
 */
static gboolean
motion_notify_event_cb (GtkWidget      *widget,
                        GdkEventMotion *event,
                        gpointer        data)
{
    GDK_THREADS_ENTER();
    guard _gg(mx);
    /* paranoia check, in case we haven't gotten a configure event */
    if (surface == NULL)
        return FALSE;

    /*
      if (event->state & GDK_BUTTON1_MASK)
      draw_brush (widget, event->x, event->y);
    */

    /* We've handled it, stop processing */
    GDK_THREADS_LEAVE();
    return TRUE;
}

static void
close_window (void)
{
    guard _gg(mx);
    if (surface) {
        cairo_surface_destroy(surface);
    }
}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
    GtkWidget *frame;

    window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "Viz!");

    g_signal_connect(window, "destroy", G_CALLBACK(close_window), NULL);

    gtk_container_set_border_width(GTK_CONTAINER(window), 8);

    frame = gtk_frame_new(NULL);
    gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_IN);
    gtk_container_add(GTK_CONTAINER(window), frame);

    drawing_area = gtk_drawing_area_new();
    /* set a minimum size */
    gtk_widget_set_size_request(drawing_area, 600, 600);

    gtk_container_add(GTK_CONTAINER(frame), drawing_area);

    /* Signals used to handle the backing surface */
    g_signal_connect(drawing_area, "draw",
                     G_CALLBACK(draw_cb), NULL);
    g_signal_connect(drawing_area,"configure-event",
                     G_CALLBACK(configure_event_cb), NULL);

    /* Event signals */
    g_signal_connect (drawing_area, "motion-notify-event",
                      G_CALLBACK (motion_notify_event_cb), NULL);
    g_signal_connect(drawing_area, "button-press-event",
                     G_CALLBACK(button_press_event_cb), NULL);

    /* Ask to receive events the drawing area doesn't normally
     * subscribe to. In particular, we need to ask for the
     * button press and motion notify events that want to handle.
     */
    gtk_widget_set_events(drawing_area, gtk_widget_get_events (drawing_area)
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_POINTER_MOTION_MASK);

    gtk_widget_show_all(window);
    gdk_threads_init();
}

/* Draws the angle of range of the sensor hit relative to the robot's current position and draws point with Cairo */
void
viz_hit(float range, float angle)
{
    GDK_THREADS_ENTER();
    guard _gg(mx);
    gtk_window_get_size(GTK_WINDOW(window), &W_WIDTH, &W_HEIGHT);
    int mid = min(W_WIDTH, W_HEIGHT) / 2;
    angle += (M_PI / 2.0);
    float dx = 0.5 * range * cos(angle);
    float dy = 0.5 * range * sin(angle);

    /*
    cout << "rr,aa; dx,dy = "
         << range << "," << angle << "; "
         << dx << "," << dy << endl;
    */

    int xx = mid + (mid*dx);
    int yy = W_HEIGHT - (mid + (mid*dy));

    /*
    cout << "ww,hh; xx,yy = "
         << ww << "," << hh << "; "
         << xx << "," << yy << endl;
    */

    draw_brush(drawing_area, xx, yy);
    GDK_THREADS_LEAVE();
}

/* 
Draws approximate location of the sensor hit and maps out representation of the board with Cairo. 
Adapted from: https://github.com/navinrahim/RoboND-Occupancy-Grid-Mapping-Algorithm/blob/master/main.cpp
*/
void
viz_pos(Cell map[][280]) 
{
    // find position in map representation and store value
    for (int row = 0; row < 280; row++) {
        for (int col = 0; col < 280; col++) {
            // double x = row * CELL_WIDTH + (CELL_WIDTH / 2);
            // double y = -(col * CELL_HEIGHT + (CELL_HEIGHT / 2));
            // add the cell's value to map
            // cout << "Cell Val: " << map[row][col] << endl;
            cout << "Draw!" << endl;
            Cell curr = map[row][col];
            
            int cell_height = row * 2.5;
            int cell_width = col * 2.5;
            draw_brush(drawing_area, cell_height, cell_width);

            cout << "Cell prob: " << curr.prob << endl;

            if (curr.prob == 0) {
                cout << "I'M GOING INTO HERE" << endl;
                draw_free(drawing_area, cell_height, cell_width);
            }
            
            if (curr.prob >= 0.9) {
                cout << "JK I'M HERE" << endl;
                draw_wall(drawing_area, cell_height, cell_width);
            }
        }
    }
}


/* Initializes the visualizer */
int
viz_run(int argc, char **argv)
{
    XInitThreads();

    GtkApplication *app;

    gtk_window_get_size(GTK_WINDOW(window), &W_WIDTH, &W_HEIGHT);
    app = gtk_application_new("site.ntuck-neu.brain", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);
    return status;
}