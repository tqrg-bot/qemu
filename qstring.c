/*
 * QString data type.
 *
 * Copyright (C) 2009 Red Hat Inc.
 *
 * Authors:
 *  Luiz Capitulino <lcapitulino@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */
#include "qobject.h"
#include "qstring.h"
#include "qemu-common.h"

static void qstring_destroy_obj(QObject *obj);
static void qstring_encode_json(const QObject *obj, QString *str);

static const QType qstring_type = {
    .code = QTYPE_QSTRING,
    .destroy = qstring_destroy_obj,
    .encode_json = qstring_encode_json,
};

/**
 * Invariant: all strings have an empty byte at the end so that
 * it is easy to convert them to C strings.
 */


/**
 * qstring_new(): Create a new empty QString
 *
 * Return strong reference.
 */
QString *qstring_new(void)
{
    QString *qstring;
    qstring = qemu_malloc(sizeof(*qstring));
    qstring->n = 0;
    qstring->alloc = 16;
    qstring->string = qemu_malloc(qstring->alloc);
    QOBJECT_INIT(qstring, &qstring_type);

    return qstring;
}

/**
 * qstring_from_str(): Create a new QString from a regular C string
 *
 * Return strong reference.
 */
QString *qstring_from_str(const char *str)
{
    QString *qstring;
    size_t n = strlen(str);

    qstring = qemu_malloc(sizeof(*qstring));
    qstring->n = n;
    qstring->alloc = n + 1;
    qstring->string = qemu_memdup(str, qstring->alloc);
    QOBJECT_INIT(qstring, &qstring_type);

    return qstring;
}

/**
 * qstring_json_from_qobject_obj(): Encode a QObject as JSON and return
 * a QString with the result.
 *
 * Return strong reference.
 */
QString *qstring_json_from_qobject_obj(const QObject *qobject)
{
    QString *qstring;

    qstring = qstring_new();
    qobject_encode_json(qobject, qstring);
    return qstring;
}


/**
 * qstring_append(): Append a regular C string to a QString
 */
void qstring_append(QString *qstring, const char *str)
{
    size_t n = strlen(str);
    size_t total = qstring->n + n + 1;

    if (total > qstring->alloc) {
        if (qstring->alloc * 2 < total) {
            qstring->alloc = total;
        } else {
            qstring->alloc *= 2;
        }
        qstring->string = qemu_realloc (qstring->string, qstring->alloc);
    }
    memcpy (qstring->string + qstring->n, str, n + 1);
    qstring->n += n;
}

/**
 * qstring_append(): Append a regular C string to a QString, escaping it
 * according to JSON syntax.
 */
void qstring_append_escaped(QString *qstring, const char *str)
{
    for (; *str; str++) {
        unsigned char ch = *str;
        switch (*str) {
        case '\f': ch = 'f'; goto backslash;
        case '\n': ch = 'n'; goto backslash;
        case '\r': ch = 'r'; goto backslash;
        case '\t': ch = 't'; goto backslash;
        case '\b': ch = 'b'; goto backslash;

        backslash:
        case '\\':
        case '\"':
            qstring_append_ch (qstring, '\\');
            break;

        default:
            if (ch < 0x20) {
                    qstring_append_ch (qstring, '\\');
                    qstring_append_ch (qstring, 'u');
                    qstring_append_ch (qstring, '0');
                    qstring_append_ch (qstring, '0');
                    qstring_append_ch (qstring, '0' + (ch >> 4));
                    ch = (ch & 15) + ((ch & 15) > 9 ? 'a' - 10 : '0');
            }
            break;
        }

        qstring_append_ch (qstring, ch);
    }
}

/**
 * qstring_append_ch(): Append a character to a QString
 */
void qstring_append_ch(QString *qstring, char c)
{
    if (qstring->n == qstring->alloc - 1) {
        if (qstring->alloc < 10) {
            qstring->alloc = 10;
        } else {
            qstring->alloc *= 2;
        }
        qstring->string = qemu_realloc (qstring->string, qstring->alloc);
    }
    qstring->string[qstring->n++] = c;
}

/**
 * qobject_to_qstring(): Convert a QObject to a QString
 */
QString *qobject_to_qstring(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QSTRING)
        return NULL;

    return container_of(obj, QString, base);
}

/**
 * qstring_get_str(): Return a pointer to the stored string
 *
 * NOTE: Should be used with caution, if the object is deallocated
 * or modified this pointer becomes invalid.
 */
const char *qstring_get_str(const QString *qstring)
{
    /* NULL-terminate it here.  */
    qstring->string[qstring->n] = 0;
    return qstring->string;
}

/**
 * qstring_destroy_obj(): Free all memory allocated by a QString
 * object
 */
static void qstring_destroy_obj(QObject *obj)
{
    QString *qs;

    assert(obj != NULL);
    qs = qobject_to_qstring(obj);
    qemu_free(qs->string);
    qemu_free(qs);
}

/**
 * qstring_encode_json(): Encode the string to JSON on a QString.
 */
static void qstring_encode_json(const QObject *obj, QString *str)
{
    QString *qstring;

    assert(obj != NULL);
    qstring = qobject_to_qstring((QObject *) obj);
    qstring_append_ch (str, '"');
    qstring_append_escaped (str, qstring_get_str(qstring));
    qstring_append_ch (str, '"');
}
