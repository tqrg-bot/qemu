/*
 * Convenience functions for bluetooth.
 *
 * Copyright (C) 2008 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu-common.h"
#include "net.h"
#include "bt.h"
#include "bt-host.h"

static int nb_hcis;
static int cur_hci;
static struct HCIInfo *hci_table[MAX_NICS];

static struct bt_vlan_s {
    struct bt_scatternet_s net;
    int id;
    struct bt_vlan_s *next;
} *first_bt_vlan;


/* Slave implementations can ignore this */
static void bt_dummy_lmp_mode_change(struct bt_link_s *link)
{
}

/* Slaves should never receive these PDUs */
static void bt_dummy_lmp_connection_complete(struct bt_link_s *link)
{
    if (link->slave->reject_reason)
        fprintf(stderr, "%s: stray LMP_not_accepted received, fixme\n",
                        __FUNCTION__);
    else
        fprintf(stderr, "%s: stray LMP_accepted received, fixme\n",
                        __FUNCTION__);
    exit(-1);
}

static void bt_dummy_lmp_disconnect_master(struct bt_link_s *link)
{
    fprintf(stderr, "%s: stray LMP_detach received, fixme\n", __FUNCTION__);
    exit(-1);
}

static void bt_dummy_lmp_acl_resp(struct bt_link_s *link,
                const uint8_t *data, int start, int len)
{
    fprintf(stderr, "%s: stray ACL response PDU, fixme\n", __FUNCTION__);
    exit(-1);
}

/* Slaves that don't hold any additional per link state can use these */
static void bt_dummy_lmp_connection_request(struct bt_link_s *req)
{
    struct bt_link_s *link = qemu_mallocz(sizeof(struct bt_link_s));

    link->slave = req->slave;
    link->host = req->host;

    req->host->reject_reason = 0;
    req->host->lmp_connection_complete(link);
}

static void bt_dummy_lmp_disconnect_slave(struct bt_link_s *link)
{
    qemu_free(link);
}

static void bt_dummy_destroy(struct bt_device_s *device)
{
    bt_device_done(device);
    qemu_free(device);
}

static int bt_dev_idx = 0;

void bt_device_init(struct bt_device_s *dev, struct bt_scatternet_s *net)
{
    memset(dev, 0, sizeof(*dev));
    dev->inquiry_scan = 1;
    dev->page_scan = 1;

    dev->bd_addr.b[0] = bt_dev_idx & 0xff;
    dev->bd_addr.b[1] = bt_dev_idx >> 8;
    dev->bd_addr.b[2] = 0xd0;
    dev->bd_addr.b[3] = 0xba;
    dev->bd_addr.b[4] = 0xbe;
    dev->bd_addr.b[5] = 0xba;
    bt_dev_idx ++;

    /* Simple slave-only devices need to implement only .lmp_acl_data */
    dev->lmp_connection_complete = bt_dummy_lmp_connection_complete;
    dev->lmp_disconnect_master = bt_dummy_lmp_disconnect_master;
    dev->lmp_acl_resp = bt_dummy_lmp_acl_resp;
    dev->lmp_mode_change = bt_dummy_lmp_mode_change;
    dev->lmp_connection_request = bt_dummy_lmp_connection_request;
    dev->lmp_disconnect_slave = bt_dummy_lmp_disconnect_slave;

    dev->handle_destroy = bt_dummy_destroy;

    dev->net = net;
    dev->next = net->slave;
    net->slave = dev;
}

void bt_device_done(struct bt_device_s *dev)
{
    struct bt_device_s **p = &dev->net->slave;

    while (*p && *p != dev)
        p = &(*p)->next;
    if (*p != dev) {
        fprintf(stderr, "%s: bad bt device \"%s\"\n", __FUNCTION__,
                        dev->lmp_name ?: "(null)");
        exit(-1);
    }

    *p = dev->next;
}

/* find or alloc a new bluetooth "VLAN" */
static struct bt_scatternet_s *qemu_find_bt_vlan(int id)
{
    struct bt_vlan_s **pvlan, *vlan;
    for (vlan = first_bt_vlan; vlan != NULL; vlan = vlan->next) {
        if (vlan->id == id)
            return &vlan->net;
    }
    vlan = qemu_mallocz(sizeof(struct bt_vlan_s));
    vlan->id = id;
    pvlan = &first_bt_vlan;
    while (*pvlan != NULL)
        pvlan = &(*pvlan)->next;
    *pvlan = vlan;
    return &vlan->net;
}

static void null_hci_send(struct HCIInfo *hci, const uint8_t *data, int len)
{
}

static int null_hci_addr_set(struct HCIInfo *hci, const uint8_t *bd_addr)
{
    return -ENOTSUP;
}

static struct HCIInfo null_hci = {
    .cmd_send = null_hci_send,
    .sco_send = null_hci_send,
    .acl_send = null_hci_send,
    .bdaddr_set = null_hci_addr_set,
};

struct HCIInfo *qemu_next_hci(void)
{
    if (cur_hci == nb_hcis)
        return &null_hci;

    return hci_table[cur_hci++];
}

struct HCIInfo *bt_hci_parse(const char *str)
{
    char *endp;
    struct bt_scatternet_s *vlan = 0;

    if (!strcmp(str, "null"))
        /* null */
        return &null_hci;
    else if (!strncmp(str, "host", 4) && (str[4] == '\0' || str[4] == ':'))
        /* host[:hciN] */
        return bt_host_hci(str[4] ? str + 5 : "hci0");
    else if (!strncmp(str, "hci", 3)) {
        /* hci[,vlan=n] */
        if (str[3]) {
            if (!strncmp(str + 3, ",vlan=", 6)) {
                vlan = qemu_find_bt_vlan(strtol(str + 9, &endp, 0));
                if (*endp)
                    vlan = 0;
            }
        } else
            vlan = qemu_find_bt_vlan(0);
        if (vlan)
           return bt_new_hci(vlan);
    }

    fprintf(stderr, "qemu: Unknown bluetooth HCI `%s'.\n", str);

    return 0;
}

static int bt_hci_add(const char *str)
{
    struct HCIInfo *hci;
    bdaddr_t bdaddr;

    if (nb_hcis >= MAX_NICS) {
        fprintf(stderr, "qemu: Too many bluetooth HCIs (max %i).\n", MAX_NICS);
        return -1;
    }

    hci = bt_hci_parse(str);
    if (!hci)
        return -1;

    bdaddr.b[0] = 0x52;
    bdaddr.b[1] = 0x54;
    bdaddr.b[2] = 0x00;
    bdaddr.b[3] = 0x12;
    bdaddr.b[4] = 0x34;
    bdaddr.b[5] = 0x56 + nb_hcis;
    hci->bdaddr_set(hci, bdaddr.b);

    hci_table[nb_hcis++] = hci;

    return 0;
}

static void bt_vhci_add(int vlan_id)
{
    struct bt_scatternet_s *vlan = qemu_find_bt_vlan(vlan_id);

    if (!vlan->slave)
        fprintf(stderr, "qemu: warning: adding a VHCI to "
                        "an empty scatternet %i\n", vlan_id);

    bt_vhci_init(bt_new_hci(vlan));
}

static struct bt_device_s *bt_device_add(const char *opt)
{
    struct bt_scatternet_s *vlan;
    int vlan_id = 0;
    char *endp = strstr(opt, ",vlan=");
    int len = (endp ? endp - opt : strlen(opt)) + 1;
    char devname[10];

    pstrcpy(devname, MIN(sizeof(devname), len), opt);

    if (endp) {
        vlan_id = strtol(endp + 6, &endp, 0);
        if (*endp) {
            fprintf(stderr, "qemu: unrecognised bluetooth vlan Id\n");
            return 0;
        }
    }

    vlan = qemu_find_bt_vlan(vlan_id);

    if (!vlan->slave)
        fprintf(stderr, "qemu: warning: adding a slave device to "
                        "an empty scatternet %i\n", vlan_id);

    if (!strcmp(devname, "keyboard"))
        return bt_keyboard_init(vlan);

    fprintf(stderr, "qemu: unsupported bluetooth device `%s'\n", devname);
    return 0;
}

int bt_parse(const char *opt)
{
    const char *endp, *p;
    int vlan;

    if (strstart(opt, "hci", &endp)) {
        if (!*endp || *endp == ',') {
            if (*endp)
                if (!strstart(endp, ",vlan=", 0))
                    opt = endp + 1;

            return bt_hci_add(opt);
       }
    } else if (strstart(opt, "vhci", &endp)) {
        if (!*endp || *endp == ',') {
            if (*endp) {
                if (strstart(endp, ",vlan=", &p)) {
                    vlan = strtol(p, (char **) &endp, 0);
                    if (*endp) {
                        fprintf(stderr, "qemu: bad scatternet '%s'\n", p);
                        return 1;
                    }
                } else {
                    fprintf(stderr, "qemu: bad parameter '%s'\n", endp + 1);
                    return 1;
                }
            } else
                vlan = 0;

            bt_vhci_add(vlan);
            return 0;
        }
    } else if (strstart(opt, "device:", &endp))
        return !bt_device_add(endp);

    fprintf(stderr, "qemu: bad bluetooth parameter '%s'\n", opt);
    return 1;
}

