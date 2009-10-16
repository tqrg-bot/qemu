#ifndef QSTRING_H
#define QSTRING_H

#include "qobject.h"

typedef struct QString {
    QObject_HEAD;
    size_t n, alloc;
    char *string;
} QString;

QString *qstring_new(void);
QString *qstring_from_str(const char *str);
const char *qstring_get_str(const QString *qstring);
void qstring_append(QString *qstring, const char *str);
void qstring_append_escaped(QString *qstring, const char *str);
void qstring_append_ch(QString *qstring, char c);
QString *qobject_to_qstring(const QObject *obj);

#endif /* QSTRING_H */
