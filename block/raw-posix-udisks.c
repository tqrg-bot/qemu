/*
 * Udisks interaction for CD-ROM unmounting
 *
 * Copyright (c) 2012 Red Hat, Inc.
 *
 * Author: Paolo Bonzini  <pbonzini@redhat.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <glib.h>
#include <glib-object.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <dbus/dbus-glib.h>
#include <dbus/dbus-glib-lowlevel.h>
#include "block/raw-posix-udisks.h"

static void udisks_find_device_by_device_file(DBusGConnection *bus,
                                              const char *path,
                                              char **object_path,
                                              GError **error)
{
    DBusGProxy *proxy;
    proxy = dbus_g_proxy_new_for_name (bus, "org.freedesktop.UDisks",
                                       "/org/freedesktop/UDisks",
                                       "org.freedesktop.UDisks");
    dbus_g_proxy_call (proxy, "FindDeviceByDeviceFile", error,
                       G_TYPE_STRING, path, G_TYPE_INVALID,
                       DBUS_TYPE_G_OBJECT_PATH, object_path, G_TYPE_INVALID);
    g_object_unref (proxy);
}

static void udisks_device_filesystem_unmount(DBusGConnection *bus,
                                             const char *object_path,
                                             GError **error)
{
    DBusGProxy *proxy;
    proxy = dbus_g_proxy_new_for_name (bus, "org.freedesktop.UDisks",
                                       object_path,
                                       "org.freedesktop.UDisks.Device");
    dbus_g_proxy_call (proxy, "FilesystemUnmount", error,
                       G_TYPE_STRV, NULL, G_TYPE_INVALID,
                       G_TYPE_INVALID);
    g_object_unref (proxy);
}

int udisks_unmount (const char *path)
{
    DBusGConnection *bus;
    char *object_path;
    GError *error;
    int ret;

    error = NULL;
    bus = dbus_g_bus_get (DBUS_BUS_SYSTEM, &error);
    if (bus == NULL) {
        g_warning ("Couldn't connect to system bus: %s", error->message);
        ret = -EACCES;
        goto out;
    }

    udisks_find_device_by_device_file(bus, path, &object_path, &error);
    if (error || !object_path) {
        ret = -ENODEV;
        goto out;
    }

    udisks_device_filesystem_unmount(bus, object_path, &error);
    if (error) {
        g_print ("Unmount failed: %s\n", error->message);
        ret = -EBUSY;
        goto out;
      }

    ret = 0; /* success */
out:
    g_free (object_path);
    if (error != NULL) {
        g_error_free (error);
    }
    if (bus != NULL) {
        dbus_g_connection_unref (bus);
    }
    return ret;
}
