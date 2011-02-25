#include <linux/types.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <sound/control.h>

#include <asm/arch/twl4030-audio.h>
#include "twl4030-audio.h"

#undef DEBUG
//#define DEBUG

#undef DPRINTK
#ifdef DEBUG
#define DPRINTK(x ...) \
	printk(KERN_INFO "<%s>: ", __FUNCTION__); \
	printk(KERN_INFO x)
#else
#define DPRINTK(x ...)
#endif

enum op {
	NOP,
	SETREG,    /* Set register arg1 to value arg2 */
	SETBIT,    /* Set bitmask arg2 of register arg1 */
	CLRBIT,    /* Clear bitmask arg2 of register arg1 */
	CLRREG,    /* Clear register arg1 */
	WAIT,      /* Delay arg1 ms */
	CODEC,     /* Codec enable/disable */
	BEGIN,     /* Begin script arg1 */
	END,       /* End of script */
	DELETEALL, /* Delete all scripts */
	NUM_OPS,
};
static const char *op_str[] = {
	"nop",
	"setreg",
	"setbit",
	"clrbit",
	"clrreg",
	"wait",
	"codec",
	"begin",
	"end",
	"deleteall",
};

struct instr {
	enum op op;
	unsigned arg1;
	unsigned arg2;
	unsigned arg3;
	unsigned arg4;
};

enum script_type {
	NOTYPE,
	INIT,      /* Settings that cannot be modified when codec is running */
	VOLUME,    /* This script sets a volume level */
};

#define INSTR_CNT_DEFAULT    50
#define SCRIPT_KEY_LEN       64
struct script {
	char key[SCRIPT_KEY_LEN];
	enum script_type type;
	union {
		struct {
			unsigned min_vol;
			unsigned max_vol;
		} volume;
		char reserved[16];
	} info;

	unsigned instr_cnt;
	unsigned instr_cnt_max;
	struct instr *instrs;    /* array */

	struct list_head next;
};

#define VOLUME_LEVELS_DEFAULT 30    /* FIXME match reality */
struct volume_script {
	char key[SCRIPT_KEY_LEN];

	struct script **scripts;    /* array */
	int levels;
	int levels_max;

	unsigned min;
	unsigned max;

	char ctl_name[SCRIPT_KEY_LEN + 10];
	struct snd_kcontrol *kctl;

	struct script *current_script;
	int current_volume;         /* store this for the get call */

	struct list_head next;
};
static unsigned long volume_script_cnt;

static struct list_head scripts;
static struct list_head volume_scripts;
static struct script *init_script;
static struct script *curr_script;
static struct snd_card *card;

/* Protect script runs */
static struct semaphore script_run_sem;

static void script_run_internal(struct script *s)
{
	struct instr *ins;
	int i;
	DPRINTK("Run script %s\n", s->key);

	down(&script_run_sem);

	for (i = 0; i < s->instr_cnt; i++) {
		ins = &s->instrs[i];
		switch (ins->op) {
			case SETREG:
				twl4030_audio_write(ins->arg1, ins->arg2);
				break;

			case SETBIT:
				twl4030_audio_bit_set(ins->arg1, ins->arg2);
				break;

			case CLRBIT:
				twl4030_audio_bit_clr(ins->arg1, ins->arg2);
				break;

			case CLRREG:
				twl4030_audio_write(ins->arg1, 0);
				break;

			case WAIT:
				msleep(ins->arg1);
				break;

			case CODEC: /* CODEC is a misnomer - this is called when a phone call is started/stopped */
				twl4030_audio_codec_phonecall_enable(ins->arg1);
				break;

			default:
				/* Should never get here, just skip the bad command */
				printk(KERN_ERR 
						"OMAP-AUDIO: unknown scripted command %d\n", 
						ins->op);
				break;
		}
	}

	up(&script_run_sem);
}

static struct volume_script * find_volume_script_by_kctl(
		struct snd_kcontrol *kctl)
{
	struct volume_script *vs = NULL;

	DPRINTK("Looking for private value %lu\n", kctl->private_value);

	list_for_each_entry(vs, &volume_scripts, next) {
		if (vs->kctl->private_value == kctl->private_value) {
			break;
		}
	}

	return vs;
}

static int mixer_volume_info(struct snd_kcontrol *kctl, 
		struct snd_ctl_elem_info *info)
{
	struct volume_script *vs;

	vs = find_volume_script_by_kctl(kctl);
	if (vs == NULL) {
		return -EINVAL;
	}

	info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	info->count = 1;
	info->value.integer.min = vs->min;
	info->value.integer.max = vs->max;

	return 0;
}

static int mixer_volume_get(struct snd_kcontrol *kctl,
		struct snd_ctl_elem_value *v)
{
	struct volume_script *vs;

	vs = find_volume_script_by_kctl(kctl);
	if (vs == NULL) {
		return -EINVAL;
	}

	v->value.integer.value[0] = vs->current_volume;

	return 0;
}

static int mixer_volume_put(struct snd_kcontrol *kctl,
		struct snd_ctl_elem_value *v)
{
	int r = 0;
	struct volume_script *vs;
	struct script *s = NULL;
	int new, i;

	vs = find_volume_script_by_kctl(kctl);
	if (vs == NULL) {
		return -EINVAL;
	}

	new = v->value.integer.value[0];
	if (new == vs->current_volume) {
		goto mixer_volume_put_end;
	}

	if (vs->current_script != NULL &&
			new >= vs->current_script->info.volume.min_vol &&
			new <= vs->current_script->info.volume.max_vol) {
		/* New level is in the range of the current settings */
		vs->current_volume = new;
		goto mixer_volume_put_end;
	}

	s = NULL;
	for (i = 0; i < vs->levels; i++) {
		s = vs->scripts[i];
		if (new >= s->info.volume.min_vol && new <= s->info.volume.max_vol) {
			break;
		}
	}

	if (s == NULL) {
		/* Uh.. there must be a hole in the programmed scripts */
		printk(KERN_ERR "OMAP-AUDIO: Volume level %d not found for %s\n",
				new, vs->ctl_name);
		printk(KERN_ERR "OMAP_AUDIO: Available levels: ");
		for (i = 0; i < vs->levels; i++) {
			printk(KERN_ERR "%u-%u ", vs->scripts[i]->info.volume.min_vol,
					vs->scripts[i]->info.volume.max_vol);
		}
		printk(KERN_ERR "\n");
		r = -EINVAL;

	} else {
		vs->current_script = s;
		vs->current_volume = new;

		script_run_internal(s);
		r = 1;
	}

mixer_volume_put_end:
	/* 1 = changed, 0 = not changed */
	return r;
}

static void add_instr(struct script *script, enum op op,
		unsigned arg1, unsigned arg2, unsigned arg3, unsigned arg4)
{
	struct instr *i;

	DPRINTK("add op=%d arg1=%#x arg2=%#x arg3=%#x arg4=%#x\n", op,
			arg1, arg2, arg3, arg4);
	DPRINTK("add script=%p instrs=%p instr_cnt=%d max=%d\n", script,
			script->instrs, script->instr_cnt, script->instr_cnt_max);

	if (script->instrs == NULL) {
		script->instrs = (struct instr *)
			kmalloc(INSTR_CNT_DEFAULT * sizeof(struct instr), GFP_KERNEL);
		script->instr_cnt = 0;
		script->instr_cnt_max = INSTR_CNT_DEFAULT;

	} else if (script->instr_cnt >= script->instr_cnt_max) {
		struct instr *bigi;

		bigi = (struct instr *)
			kmalloc(script->instr_cnt_max * 2 * sizeof(struct instr),
					GFP_KERNEL);
		memcpy(bigi, script->instrs, 
				script->instr_cnt_max * sizeof(struct instr));
		kfree(script->instrs);

		script->instrs = bigi;
		script->instr_cnt_max *= 2;
	}

	i = &script->instrs[script->instr_cnt];

	i->op = op;
	i->arg1 = arg1;
	i->arg2 = arg2;
	i->arg3 = arg3;
	i->arg4 = arg4;

	script->instr_cnt += 1;
}

static char *parse_line(char *_buf, int *nargs, enum op *op, 
		char **arg1, char **arg2, char **arg3, char **arg4)
{
	int r = 0;
	char *buf = _buf;
	char **args[4] = { arg1, arg2, arg3, arg4 };
	enum { SPACE, DONE, START, S_OP, /* S_A1, S_A2, S_A3, S_A4 */ } 
		state, prev_state, next_state;
	char *token = NULL;
#ifdef DEBUG
	int i;
#endif

	state = START;
	prev_state = next_state = state;
	while (state != DONE && *buf != '\0') {
		DPRINTK("state=%d char: %c (%#x)\n", state, *buf, *buf);
		switch (*buf) {
			case ' ':
			case '\t':
			case '\n':
			case '#':
			case '\0':
			case 0xd: {
				int comment = (*buf == '#');
				int end = (*buf == '\n' || *buf == '\0' || comment) ? 1 : 0;

				if (state > START) {
					int i;

					/* Add terminating NULL for last token */
					*buf = '\0';

					DPRINTK("char=%#x token=%s\n", *buf, token);

					/* Handle the token */
					if (state == S_OP) {
						for (i = 0; i < NUM_OPS; i++) {
							if (strcmp(op_str[i], token) == 0) {
								*op = i;
								break;
							}
						}
						if (i >= NUM_OPS) {
							/* Unrecognized instruction */
							r = -1;
							goto parse_line_end;
						}

					} else if (state > S_OP) {
						*args[state - S_OP - 1] = token;
					}

					prev_state = state;
				}

				if (end) {
					if (state == SPACE) {
						state = prev_state;
						*nargs = state - S_OP;
					} else if (state == START) {
						*op = NOP;
						*nargs = 0;
					} else {
						*nargs = state - S_OP;
					}

					/* This is a comment, skip to the end */
					if (comment) {
						while (*buf != '\n' && *buf != '\0') {
							buf += 1;
						}
					}

					next_state = DONE;

				} else {
					next_state = SPACE;
				}

				break;
			}

			default:
				if (state == SPACE) {
					next_state = prev_state + 1;
					token = buf;
				}
				if (state == START) {
					token = buf;
					next_state = S_OP;
				}
				break;
		}
		buf += 1;
		state = next_state;
	}

parse_line_end:
#ifdef DEBUG
	DPRINTK("r=%d buf=%p op=%d nargs=%d\n", r, buf, *op, *nargs);
	for (i = 0; i < *nargs; i++) {
		DPRINTK("arg%d=%s\n", i + 1, *args[i]);
	}
#endif

	if (r < 0) {
		return (char *)r;
	} else {
		return buf;
	}
}

static struct script *find_script(const char *name)
{
	struct script *s;
	struct script *found = NULL;

	list_for_each_entry(s, &scripts, next) {
		if (strncmp(s->key, name, SCRIPT_KEY_LEN) == 0) {
			found = s;
			break;
		}
	}

	return found;
}

static struct volume_script *find_volume_script(const char *name)
{
	struct volume_script *s;
	struct volume_script *found = NULL;

	list_for_each_entry(s, &volume_scripts, next) {
		if (strncmp(s->key, name, SCRIPT_KEY_LEN) == 0) {
			found = s;
			break;
		}
	}

	return found;
}

static void debug_print_script(struct script *s)
{
	int i;
	if (s->instr_cnt > 0) {
		for (i = 0; i < s->instr_cnt; i++) {
			struct instr *ins = &s->instrs[i];
			printk(KERN_INFO "%03d: ", i);
			printk(KERN_INFO "%10s %#08x %#08x %#08x %#08x\n",
					op_str[ins->op], ins->arg1, ins->arg2, 
					ins->arg3, ins->arg4);
		}
	}
}

static void debug_show_all(void)
{
	struct script *s;
	int printed = 0;

	if (init_script != NULL) {
		printed += 1;
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "begin script INIT:\n");
		printk(KERN_INFO "instr_cnt=%d max=%d\n", 
				init_script->instr_cnt, init_script->instr_cnt_max);
		debug_print_script(init_script);
	}
	list_for_each_entry(s, &scripts, next) {
		printed += 1;
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "begin script %s:\n", s->key);
		printk(KERN_INFO "name=%s type=%d ", s->key, s->type);
		if (s->type == VOLUME) {
			printk(KERN_INFO "min_vol=%u max_vol=%u\n",
					s->info.volume.min_vol, s->info.volume.max_vol);
		} else {
			printk(KERN_INFO "\n");
		}
		printk(KERN_INFO "instr_cnt=%d max=%d\n", 
				s->instr_cnt, s->instr_cnt_max);
		debug_print_script(s);
	}

	if (printed) {
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "Printed %d scripts\n", printed);
		printk(KERN_INFO "-----------------------------\n");
	} else {
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "No scripts\n");
		printk(KERN_INFO "-----------------------------\n");
	}
}

static void debug_list_all(void)
{
	struct script *s;
	int printed = 0;

	if (init_script != NULL) {
		printed += 1;
		printk(KERN_INFO "%3d: INIT (%d instrs)\n", 
				printed, init_script->instr_cnt);
	}
	list_for_each_entry(s, &scripts, next) {
		printed += 1;
		printk(KERN_INFO "%3d: %s (%d instrs)\n", 
				printed, s->key, s->instr_cnt);
	}
	printk(KERN_INFO "-----------------------------\n");
	printk(KERN_INFO "Printed %d scripts\n", printed);
}

static void debug_show_volume(void)
{
	struct volume_script *vs;
	int printed = 0;

	list_for_each_entry(vs, &volume_scripts, next) {
		printed += 1;
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "begin volume script %s:\n", vs->key);
		printk(KERN_INFO "control name=%s priv=%lu\n",
				vs->ctl_name, vs->kctl->private_value);
		printk(KERN_INFO "min=%u max=%u levels=%d levels_max=%d\n",
				vs->min, vs->max, vs->levels, vs->levels_max);
		if (vs->levels > 0) {
			int i;
			for (i = 0; i < vs->levels; i++) {
				struct script *s = vs->scripts[i];
				printk(KERN_INFO "**\n");
				printk(KERN_INFO "level %d: script name=%s min=%u max=%u\n",
						i, s->key, s->info.volume.min_vol, 
						s->info.volume.max_vol);
				debug_print_script(s);
			}
		}
	}

	if (printed) {
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "Printed %d volume scripts\n", printed);
		printk(KERN_INFO "-----------------------------\n");
	} else {
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "No volume scripts\n");
		printk(KERN_INFO "-----------------------------\n");
	}
}

static int handle_begin(int line, unsigned nargs, char **args)
{
	int r = 0;
	struct volume_script *vs;
	unsigned min_vol, max_vol;

	curr_script = find_script(args[0]);
	if (curr_script == NULL) {
		if (nargs > 1) {
			if (strcmp("init", args[1]) == 0) {
				/* Only one init script */
				if (init_script != NULL) {
					curr_script = init_script;
					curr_script->instr_cnt = 0;

				} else {
					curr_script = kzalloc(sizeof(struct script), GFP_KERNEL);
					/* Name argument is dropped for INIT scripts */
					curr_script->type = INIT;

					init_script = curr_script;
				}

				/* INIT scripts are not added to the list of scripts */

			} else if (strcmp("volume", args[1]) == 0) {
				int i;

				vs = find_volume_script(args[0]);
				if (vs == NULL) {
					int i;
					char *n;
					struct snd_kcontrol_new nkctl;

					vs = kzalloc(sizeof(struct volume_script), GFP_KERNEL);
					vs->scripts = kmalloc(
							VOLUME_LEVELS_DEFAULT * sizeof(struct script *), 
							GFP_KERNEL);
					vs->levels_max = VOLUME_LEVELS_DEFAULT;

					n = args[0];
					strncpy(vs->key, n, SCRIPT_KEY_LEN);
					/* Crappy strncpy to replace the underscores */
					for (i = 0; i < sizeof(vs->ctl_name); i++) {
						if (n[i] == '\0') {
							break;
						}
						if (n[i] == '_') {
							vs->ctl_name[i] = ' ';
						} else {
							vs->ctl_name[i] = n[i];
						}
					}
					snprintf(&vs->ctl_name[i], sizeof(vs->ctl_name) - i, 
							" Volume");

					DPRINTK("New volume script: name=%s ctl_name=%s\b", 
							vs->key, vs->ctl_name);

					/* Export the ALSA mixer volume control */
					memset(&nkctl, 0, sizeof(struct snd_kcontrol_new));
					nkctl.name = vs->ctl_name;
					nkctl.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
					nkctl.index = 0;
					nkctl.access = SNDRV_CTL_ELEM_ACCESS_READWRITE;
					nkctl.info = mixer_volume_info;
					nkctl.get = mixer_volume_get;
					nkctl.put = mixer_volume_put;
					nkctl.private_value = volume_script_cnt;

					vs->kctl = snd_ctl_new1(&nkctl, card);
					r = snd_ctl_add(card, vs->kctl);
					if (r) {
						printk(KERN_ERR 
								"Failed to add mixer control %s (%d)\n",
								nkctl.name, r);
						/* FIXME clean up or not? */
					}

					list_add(&vs->next, &volume_scripts);
					volume_script_cnt += 1;
				}

				if (vs->levels == vs->levels_max) {
					/* Grow the array */
					struct script **bs;
					size_t bigsize;

					bigsize = vs->levels_max * 2 * sizeof(struct script *);
					bs = kmalloc(bigsize, GFP_KERNEL);
					memcpy(bs, vs->scripts, bigsize);

					kfree(vs->scripts);
					vs->scripts = bs;
					vs->levels_max *= 2;
				}

				min_vol = simple_strtoul(args[2], NULL, 0);
				max_vol = simple_strtoul(args[3], NULL, 0);

				/* Check the arguments */
				if (!(min_vol < max_vol) || min_vol > 1000 || max_vol > 1000) {
					r = -1;
					goto handle_begin_fail;
				}

				for (i = 0; i < vs->levels; i++) {
					struct script *s;
					s = vs->scripts[i];

					if (min_vol == s->info.volume.min_vol && 
							max_vol == s->info.volume.max_vol) {
						/* Found script with the same volume range,
						 * reset it */
						curr_script = s;
						curr_script->instr_cnt = 0;
						break;
					}

					if ((min_vol >= s->info.volume.min_vol &&
								min_vol <= s->info.volume.max_vol) ||
							(max_vol >= s->info.volume.min_vol &&
							 max_vol <= s->info.volume.max_vol)) {
						printk(KERN_ERR 
								"OMAP-AUDIO: Line %d: Ranges overlap\n", line);
						r = -1;
						goto handle_begin_fail;
					}
				}

				/* Allocate a new struct script if necessary */
				if (curr_script == NULL) {
					curr_script = kzalloc(sizeof(struct script), GFP_KERNEL);
					curr_script->type = VOLUME;
					curr_script->info.volume.min_vol = min_vol;
					curr_script->info.volume.max_vol = max_vol;

					/* Rename the script with the ranges embedded in it */
					snprintf(curr_script->key, SCRIPT_KEY_LEN,
							"%.*s_%u-%u", SCRIPT_KEY_LEN - 20, args[0], 
							min_vol, max_vol);

					/* Adjust the range of this control */
					if (min_vol < vs->min) {
						vs->min = min_vol;
					}
					if (max_vol > vs->max) {
						vs->max = max_vol;
					}

					/* Add it to the list of available volume levels */
					vs->scripts[vs->levels] = curr_script;
					vs->levels += 1;
				}

				DPRINTK("name=%s min_vol=%d max_vol=%d\n", 
						curr_script->key,
						curr_script->info.volume.min_vol,
						curr_script->info.volume.max_vol);

				list_add(&curr_script->next, &scripts);

			} else {
				printk(KERN_INFO 
						"OMAP-AUDIO: Line %d: unknown type\n", line);
				r = -1;
			}

		} else {
			/* New script without type */
			curr_script = kzalloc(sizeof(struct script), GFP_KERNEL);
			strncpy(curr_script->key, args[0], SCRIPT_KEY_LEN);

			curr_script->type = NOTYPE;

			list_add(&curr_script->next, &scripts);
		}
	} else {
		/* Delete previous script */
		curr_script->instr_cnt = 0;
	}

	return r;

handle_begin_fail:
	list_del(&curr_script->next);
	kfree(curr_script->instrs);
	kfree(curr_script);
	curr_script = NULL;

	return r;
}

static int handle_regop(int line, enum op op, unsigned nargs, char **args)
{
	int r = 0;
	unsigned reg, val;

	if (curr_script == NULL) {
		printk(KERN_INFO "OMAP-AUDIO: Line %d: No script for op\n", line);
		goto handle_regop_end;
	}

	if (((op == CLRREG) && nargs < 1) || ((op != CLRREG) && nargs < 2)) {
		printk(KERN_INFO "OMAP-AUDIO: Line %d: Not enough arguments\n", line);
		r = -1;
		goto handle_regop_end;
	}

	reg = simple_strtoul(args[0], NULL, 0);
	val = simple_strtoul(args[1], NULL, 0);

	switch (op) {
		case CLRREG:
			val = 0;
			/* Intentional fall-through */

		case CLRBIT:
		case SETREG:
		case SETBIT:
			add_instr(curr_script, op, reg, val, 0, 0);
			break;

		default:
			/* Should never get here */
			break;
	}

handle_regop_end:
	return r;
}

static ssize_t script_store(struct device *dev,
		struct device_attribute *attr, const char *_buf, size_t count)
{
	int r = 0;
	int line;

	char *buf = (char *)_buf;
	enum op op = NUM_OPS;
	char *args[4];
	int nargs = -1;

	if (_buf == NULL) {
		return 0;
	}

	line = 0;
	while (buf < _buf + count) {
		line += 1;

		DPRINTK("buf=%p _buf=%p count=%d\n", buf, _buf, count);
		buf = parse_line(buf, &nargs, &op, 
				&args[0], &args[1], &args[2], &args[3]);
		if ((int)buf == -1) {
			printk(KERN_INFO "OMAP-AUDIO: Line %d: parse error\n",
					line);
			r = -EIO;
			goto script_store_fail;
		}

		switch (op) {
			case BEGIN:
				if (handle_begin(line, nargs, args) < 0) {
					r = -EIO;
					goto script_store_fail;
				} else {
					DPRINTK("New script %s\n", curr_script->key);
				}
				break;

			case END:
				if (curr_script->type == INIT) {
					/* Run the init script once it's been programmed */
					script_run_internal(curr_script);
				}
				curr_script = NULL;
				break;

			case WAIT: {
				unsigned timeout = simple_strtoul(args[0], NULL, 0);

				if (curr_script == NULL) {
					printk(KERN_INFO 
							"OMAP-AUDIO: Line %d: No script for op\n", 
							line);
					continue;
				}
				add_instr(curr_script, op, timeout, 0, 0, 0);
				break;
			}

			case CODEC: {
				int on = (simple_strtoul(args[0], NULL, 0) != 0) ? 1 : 0;
				if (curr_script == NULL) {
					printk(KERN_INFO 
							"OMAP-AUDIO: Line %d: No script for op\n", 
							line);
					continue;
				}
				add_instr(curr_script, op, on, 0, 0, 0);
				break;
			}

			case CLRREG:
			case CLRBIT:
			case SETREG:
			case SETBIT:
				handle_regop(line, op, nargs, args);
				break;

			case DELETEALL: {
				struct script *s, *t;
				struct volume_script *vs, *vt;

				/* It resets the state also */
				curr_script = NULL;

				list_for_each_entry_safe(s, t, &scripts, next) {
					DPRINTK("Delete script %s\n", s->key);
					list_del(&s->next);
					kfree(s->instrs);
					kfree(s);
				}

				list_for_each_entry_safe(vs, vt, &volume_scripts, next) {
					DPRINTK("Delete volume script %s\n", vs->key);
					snd_ctl_remove(card, vs->kctl);
					kfree(s);
				}
				break;
			}

			case NOP:
				/* Nothing to do for NOP */
				break;

			/* Invalid op code checked in parse_line() so should never
			 * get here. */
			default:
				r = -EIO;
				goto script_store_fail;
		}
	}

	return count;
	
script_store_fail:
	return r;
}

static ssize_t script_run(struct device *dev,
		struct device_attribute *attr, const char *_buf, size_t count)
{
	int r = 0;
	struct script *s;
	char sname[SCRIPT_KEY_LEN];
	int buflen;

	/* Strip the newline */
	buflen = (int)strchr(_buf, '\n');
	if (buflen == 0) {
		s = find_script(_buf);
	} else {
		buflen = (char *)buflen - _buf;
		buflen = buflen < SCRIPT_KEY_LEN ? buflen : SCRIPT_KEY_LEN;
		strncpy(sname, _buf, buflen);
		sname[buflen] = '\0';
		s = find_script(sname);
	}

	if (s == NULL) {
		printk(KERN_ERR "OMAP-AUDIO: script %s not found\n", _buf);
		r = -1;

	} else {
		script_run_internal(s);
	}

	return count;
}

int script_run_init(void)
{
	int r = 0;

	if (init_script == NULL) {
		r = -1;
	} else {
		/* FIXME toggle codec power before and after */
		script_run_internal(init_script);
	}

	return r;
}

//#ifdef DEBUG
static ssize_t script_debug(struct device *dev,
		struct device_attribute *attr, const char *_buf, size_t count)
{
	char buf[SCRIPT_KEY_LEN];
	int buflen;

	DPRINTK("_buf=%s\n", _buf);

	/* Strip the newline */
	buflen = (int)strchr(_buf, '\n');
	if (buflen == 0) {
		DPRINTK("print 1\n");
		strncpy(buf, _buf, SCRIPT_KEY_LEN);
		buf[SCRIPT_KEY_LEN - 1] = '\0';
	} else {
		DPRINTK("buflen=%d\n", buflen);
		buflen = (char *)buflen - _buf;
		strncpy(buf, _buf, buflen);
		buf[buflen] = '\0';
	}

	if (strcmp("#all", buf) == 0) {
		debug_show_all();
	} else if (strcmp("#list", buf) == 0) {
		debug_list_all();
	} else if (strcmp("#volume", buf) == 0) {
		debug_show_volume();
	} else if (strcmp("#init", buf) == 0) {
		printk(KERN_INFO "-----------------------------\n");
		printk(KERN_INFO "begin script INIT:\n");
		printk(KERN_INFO "instr_cnt=%d max=%d\n", 
				init_script->instr_cnt, init_script->instr_cnt_max);
		debug_print_script(init_script);
		printk(KERN_INFO "-----------------------------\n");
	} else {
		struct script *s;
		s = find_script(buf);
		if (s == NULL) {
			printk(KERN_INFO "Script %s not found\n", buf);
		} else {
			printk(KERN_INFO "-----------------------------\n");
			printk(KERN_INFO "begin script %s:\n", s->key);
			printk(KERN_INFO "name=%s type=%d ", s->key, s->instr_cnt);
			if (s->type == VOLUME) {
				printk(KERN_INFO "min_vol=%u max_vol=%u\n",
						s->info.volume.min_vol, s->info.volume.max_vol);
			} else {
				printk(KERN_INFO "\n");
			}
			printk(KERN_INFO "instr_cnt=%d max=%d\n", 
					s->instr_cnt, s->instr_cnt_max);
			debug_print_script(s);
			printk(KERN_INFO "-----------------------------\n");
		}
	}

	return count;
}
//#endif

DEVICE_ATTR(scinit, S_IWUSR, NULL, script_store);
DEVICE_ATTR(scrun, S_IWUSR, NULL, script_run);
//#ifdef DEBUG
DEVICE_ATTR(scdebug, S_IWUSR, NULL, script_debug);
//#endif

int script_init(struct device *dev, struct snd_card *c)
{
	int r = 0;

	init_MUTEX(&script_run_sem);

	INIT_LIST_HEAD(&scripts);
	INIT_LIST_HEAD(&volume_scripts);

	card = c;

	r = device_create_file(dev, &dev_attr_scinit);
	if (r) {
		printk(KERN_ERR "OMAP-AUDIO: failed to init triton scripting\n");
	}
	r = device_create_file(dev, &dev_attr_scrun);
	if (r) {
		printk(KERN_ERR "OMAP-AUDIO: failed to init triton scripting\n");
	}
//#ifdef DEBUG
	r = device_create_file(dev, &dev_attr_scdebug);
	if (r) {
		printk(KERN_ERR 
				"OMAP-AUDIO: failed to create scripting debug sysfs entry\n");
	}
//#endif

	return r;
}
