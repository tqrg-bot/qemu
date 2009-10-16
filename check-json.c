/*
 * QObject JSON unit-tests.
 *
 * Copyright (C) 2009 Red Hat Inc.
 *
 * Authors:
 *  Paolo Bonzini <pbonzini@redhat.com>
 */
#include <check.h>

#include "qint.h"
#include "qdict.h"
#include "qlist.h"
#include "qstring.h"
#include "qemu-common.h"

#define json_expect(qobj, expected)                             \
  do {                                                          \
    QString *result = qstring_json_from_qobject (qobj);         \
    fail_unless (!strcmp (qstring_get_str (result), expected)); \
    QDECREF(result);                                            \
  } while (0)

START_TEST(qint_json_test)
{
    QInt *qi;

    qi = qint_from_int(42);
    json_expect(qi,"42");
    QDECREF(qi);
}
END_TEST

START_TEST(qstr_json_test)
{
    const char *str = "\\\"\b\n\r\f\t\v\a\x19X\xc3\x80";
    QString *qstr;

    qstr = qstring_from_str(str);
    json_expect(qstr,"\"\\\\\\\"\\b\\n\\r\\f\\t\\u000b\\u0007\\u0019X\xc3\x80\"");
    QDECREF(qstr);
}
END_TEST

START_TEST(qdict_json_test)
{
    QDict *qdict;

    qdict = qdict_new();
    json_expect(qdict,"{}");
    qdict_put_obj(qdict, "", QOBJECT(qint_from_int(42)));
    json_expect(qdict,"{\"\":42}");
    qdict_put_obj(qdict, "\t", QOBJECT(qint_from_int(0)));
    json_expect(qdict,"{\"\\t\":0,\"\":42}");
    qdict_put_obj(qdict, "\xc3\x80", QOBJECT(qstring_from_str("foo\n")));
    json_expect(qdict,"{\"\xc3\x80\":\"foo\\n\",\"\\t\":0,\"\":42}");
    QDECREF(qdict);
}
END_TEST

START_TEST(qlist_json_test)
{
    QList *qlist;

    qlist = qlist_new();
    json_expect(qlist,"[]");
    qlist_append(qlist, qint_from_int(42));
    json_expect(qlist,"[42]");
    qlist_append(qlist, qstring_from_str("bar\t"));
    json_expect(qlist,"[42,\"bar\\t\"]");
    QDECREF(qlist);
}
END_TEST

START_TEST(nested_json_test)
{
    QDict *qdict_in, *qdict_out;
    QList *qlist;

    qdict_in = qdict_new();
    qdict_put_obj(qdict_in, "in", QOBJECT(qint_from_int(42)));
    qlist = qlist_new();
    qlist_append(qlist, qdict_in);
    qdict_out = qdict_new();
    qdict_put_obj(qdict_out, "out", QOBJECT(qlist));
    json_expect(qdict_out,"{\"out\":[{\"in\":42}]}");
    QDECREF(qdict_out);
}
END_TEST

static Suite *QObject_json_suite(void)
{
    Suite *s;
    TCase *qobject_json_tcase;

    s = suite_create("QObject JSON suite");

    qobject_json_tcase = tcase_create("Public Interface");
    suite_add_tcase(s, qobject_json_tcase);
    tcase_add_test(qobject_json_tcase, qint_json_test);
    tcase_add_test(qobject_json_tcase, qstr_json_test);
    tcase_add_test(qobject_json_tcase, qdict_json_test);
    tcase_add_test(qobject_json_tcase, qlist_json_test);
    tcase_add_test(qobject_json_tcase, nested_json_test);

    return s;
}

int main(void)
{
        int nf;
        Suite *s;
        SRunner *sr;

        s = QObject_json_suite();
        sr = srunner_create(s);

        srunner_run_all(sr, CK_NORMAL);
        nf = srunner_ntests_failed(sr);
        srunner_free(sr);

        return (nf == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
