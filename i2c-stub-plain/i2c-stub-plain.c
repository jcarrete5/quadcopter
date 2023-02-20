#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include <linux/i2c.h>

#define func_pr_info(fmt, ...) pr_info("%s: " fmt, __func__, ##__VA_ARGS__)
#define func_pr_err(fmt, ...) pr_err("%s: " fmt, __func__, ##__VA_ARGS__)

static unsigned short bus_number = 0;
module_param(bus_number, ushort, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(bus_number, "Bus number");

#define MAX_CHIPS 10
static unsigned short chip_addr[MAX_CHIPS];
module_param_array(chip_addr, ushort, NULL, S_IRUGO);
MODULE_PARM_DESC(chip_addr, "Chip addresses (up to 10, between 0x03 and 0x77)");

#define MAX_REGISTER 256
struct i2c_stub_plain_chip {
	u8 register_data[MAX_REGISTER];
};

static struct i2c_stub_plain_chip *i2c_stub_plain_chips;
static int i2c_stub_plain_chips_nr;

struct i2c_stub_plain_chip *i2c_stub_plain_allocate_chips(int nr)
{
	return kcalloc(i2c_stub_plain_chips_nr,
		       sizeof(struct i2c_stub_plain_chip), GFP_KERNEL);
}

static void i2c_stub_plain_free_chips(void) { kfree(i2c_stub_plain_chips); }

static struct i2c_stub_plain_chip *
i2c_stub_plain_get_chip_by_addr(const u8 addr)
{
	int i;
	for (i = 0; i < i2c_stub_plain_chips_nr; i++) {
		if (addr == chip_addr[i]) {
			return i2c_stub_plain_chips + i;
		}
	}
	return NULL;
}

static u32 i2c_stub_plain_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

int i2c_stub_plain_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			       int num)
{
	bool was_last_msg_write = false;
	int active_register_address = 0;

	struct i2c_stub_plain_chip *chip;
	int msg_i;
	int buf_i;

	func_pr_info("num=%d\n", num);
	for (msg_i = 0; msg_i < num; ++msg_i) {
		struct i2c_msg *msg = msgs + msg_i;
		func_pr_info("msg={.addr=%#x, .flags=%#x, .len=%u, .buf=%p}\n",
			     msg->addr, msg->flags, msg->len, msg->buf);

		chip = i2c_stub_plain_get_chip_by_addr(msg->addr);

		buf_i = 0;
		if (msg->flags & I2C_M_RD) { // read
			while (buf_i < msg->len) {
				if (active_register_address >= MAX_REGISTER) {
					func_pr_err("active_register_address(%"
						    "d) >= MAX_REGISTER\n",
						    active_register_address);
					return -1;
				}
				msg->buf[buf_i++] =
				    chip->register_data
					[active_register_address++];
			}
			was_last_msg_write = false;
		} else { // write
			if (!was_last_msg_write) {
				active_register_address = msg->buf[buf_i++];
			}
			while (buf_i < msg->len) {
				if (active_register_address >= MAX_REGISTER) {
					func_pr_err("active_register_address(%"
						    "d) >= MAX_REGISTER\n",
						    active_register_address);
					return -1;
				}
				chip->register_data[active_register_address++] =
				    msg->buf[buf_i++];
			}
			was_last_msg_write = true;
		}
	}
	return msg_i;
}

static const struct i2c_algorithm i2c_stub_plain_algorithm = {
    .functionality = i2c_stub_plain_functionality,
    .master_xfer = i2c_stub_plain_master_xfer};

static struct i2c_adapter i2c_stub_plain_adapter = {
    .owner = THIS_MODULE,
    .class = I2C_CLASS_HWMON | I2C_CLASS_SPD,
    .algo = &i2c_stub_plain_algorithm,
    .name = "Plain I2C stub driver",
};

static int __init i2c_stub_plain_stub_init(void)
{
	int i;
	int ret;

	if (!chip_addr[0]) {
		pr_err("missing chip_addr\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX_CHIPS && chip_addr[i]; i++) {
		if (chip_addr[i] < 0x03 || chip_addr[i] > 0x77) {
			pr_err("invalid chip address %#x\n", chip_addr[i]);
			return -EINVAL;
		}

		pr_info("chip at chip_addr=%#x\n", chip_addr[i]);
	}

	i2c_stub_plain_chips_nr = i;
	i2c_stub_plain_chips =
	    i2c_stub_plain_allocate_chips(i2c_stub_plain_chips_nr);
	if (!i2c_stub_plain_chips)
		return -ENOMEM;

	func_pr_info("adding adapter\n");
	i2c_stub_plain_adapter.nr = bus_number;
	if ((ret = i2c_add_numbered_adapter(&i2c_stub_plain_adapter))) {
		goto fail_free;
	}

	return 0;

fail_free:
	i2c_stub_plain_free_chips();
	return ret;
}
module_init(i2c_stub_plain_stub_init);

static void __exit i2c_stub_plain_stub_exit(void)
{
	func_pr_info("deleting adapter\n");
	i2c_del_adapter(&i2c_stub_plain_adapter);
	i2c_stub_plain_free_chips();
}
module_exit(i2c_stub_plain_stub_exit);

MODULE_AUTHOR("Derek McBlane <mcblanederek@gmail.com>");
MODULE_DESCRIPTION("I2C stub driver for plain I2C protocol based on i2c-stub");
MODULE_LICENSE("GPL");
