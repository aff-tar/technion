/***************************************************************************
 *            my_ioctl_module.c
 *
 *  Dec 20 2004 - Mar 13 2005
 *  Copyright  2005  Ilia Lin and Yaniv Biton
 *  ilia@mlin.info     syanivbi@t2.technion.ac.il
 ****************************************************************************/
#define HERE 0xe
#define PROC_ENTRY_FILENAME "my_ioctl_"
#define MESSAGE_LENGTH 1024
#define OUTPUT_BUF_SIZE 1024

#define DR6_BT  0x00008000
#define DR6_BS  0x00004000
#define DR6_BD  0x00002000
#define DR6_B3  0x00000008
#define DR6_B2  0x00000004
#define DR6_B1  0x00000002
#define DR6_B0  0x00000001
#define DR6_DR_MASK  0x0000000F
#define DR_STEP         (0x4000)	/* single-step */
#define DR_SWITCH       (0x8000)	/* task switch */

#define DR7_RW_VAL(dr, drnum) 								\
       (((dr) >> (16 + (4 * (drnum)))) & 0x3)

#define DR7_RW_SET(dr, drnum, rw)                           \
       do {                                                 \
	       (dr) &= ~(0x3 << (16 + (4 * (drnum))));         	\
	       (dr) |= (((rw) & 0x3) << (16 + (4 * (drnum)))); 	\
       } while (0)

#define DR7_RW0(dr)		DR7_RW_VAL(dr, 0)
#define DR7_RW0SET(dr,rw)	DR7_RW_SET(dr, 0, rw)
#define DR7_RW1(dr)		DR7_RW_VAL(dr, 1)
#define DR7_RW1SET(dr,rw)	DR7_RW_SET(dr, 1, rw)
#define DR7_RW2(dr)		DR7_RW_VAL(dr, 2)
#define DR7_RW2SET(dr,rw)	DR7_RW_SET(dr, 2, rw)
#define DR7_RW3(dr)		DR7_RW_VAL(dr, 3)
#define DR7_RW3SET(dr,rw)	DR7_RW_SET(dr, 3, rw)


#define DR7_LEN_VAL(dr, drnum)								\
       (((dr) >> (18 + (4 * (drnum)))) & 0x3)

#define DR7_LEN_SET(dr, drnum, rw)							\
       do {                                                	\
	       (dr) &= ~(0x3 << (18 + (4 * (drnum))));         	\
	       (dr) |= (((rw) & 0x3) << (18 + (4 * (drnum)))); 	\
       } while (0)

#define DR7_LEN0(dr)		DR7_LEN_VAL(dr, 0)
#define DR7_LEN0SET(dr,len)	DR7_LEN_SET(dr, 0, len)
#define DR7_LEN1(dr)		DR7_LEN_VAL(dr, 1)
#define DR7_LEN1SET(dr,len)	DR7_LEN_SET(dr, 1, len)
#define DR7_LEN2(dr)		DR7_LEN_VAL(dr, 2)
#define DR7_LEN2SET(dr,len)	DR7_LEN_SET(dr, 2, len)
#define DR7_LEN3(dr)		DR7_LEN_VAL(dr, 3)
#define DR7_LEN3SET(dr,len)	DR7_LEN_SET(dr, 3, len)

#define DR7_G0(dr)    (((dr)>>1)&0x1)
#define DR7_G0SET(dr) ((dr) |= 0x2)
#define DR7_G0CLR(dr) ((dr) &= ~0x2)
#define DR7_G1(dr)    (((dr)>>3)&0x1)
#define DR7_G1SET(dr) ((dr) |= 0x8)
#define DR7_G1CLR(dr) ((dr) &= ~0x8)
#define DR7_G2(dr)    (((dr)>>5)&0x1)
#define DR7_G2SET(dr) ((dr) |= 0x20)
#define DR7_G2CLR(dr) ((dr) &= ~0x20)
#define DR7_G3(dr)    (((dr)>>7)&0x1)
#define DR7_G3SET(dr) ((dr) |= 0x80)
#define DR7_G3CLR(dr) ((dr) &= ~0x80)

#define DR7_L0(dr)    (((dr))&0x1)
#define DR7_L0SET(dr) ((dr) |= 0x1)
#define DR7_L0CLR(dr) ((dr) &= ~0x1)
#define DR7_L1(dr)    (((dr)>>2)&0x1)
#define DR7_L1SET(dr) ((dr) |= 0x4)
#define DR7_L1CLR(dr) ((dr) &= ~0x4)
#define DR7_L2(dr)    (((dr)>>4)&0x1)
#define DR7_L2SET(dr) ((dr) |= 0x10)
#define DR7_L2CLR(dr) ((dr) &= ~0x10)
#define DR7_L3(dr)    (((dr)>>6)&0x1)
#define DR7_L3SET(dr) ((dr) |= 0x40)
#define DR7_L3CLR(dr) ((dr) &= ~0x40)

#define DR7_GD          0x00002000	/* General Detect Enable */
#define DR7_GE          0x00000200	/* Global exact */
#define DR7_LE          0x00000100	/* Local exact */

#define DR_TYPE_EXECUTE	0x0
#define DR_TYPE_WRITE	0x1
#define DR_TYPE_IO	0x2
#define DR_TYPE_RW	0x3

//#define PAGE_SHIFT 12                         /* PAGE_SHIFT determines the page size */
//#define PAGE_SIZE (1UL << PAGE_SHIFT)
#define PAGE_MASK (~(PAGE_SIZE-1))

#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>	/* Necessary because we use the proc fs */
#include <asm/uaccess.h>	/* for get_user and put_user */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <asm/system.h>
#include <asm-i386/processor.h>
#include <asm-i386/desc.h>
#include <asm-i386/pgtable.h>


static __u32 idt_addr = 0;
static __u32 gdt_addr = 0;
static __u32 new_addr = 0;
static __u32 cr0, cr3, cr4, dr0, dr1, dr2, dr3, dr6, dr7;
static __u16 tr, cs, ds, es, ss, fs, gs, ldtr;

static int rv;
static __u32 dbnr;

static __u32 original_handler[32];
static int exeption_count[32];
static char Message[2048];

// IDT Descriptor format.

typedef struct
{
	__u64 	off_low		:16,
			sel			:16,
			reserved	:8,
			type		:4,
			system		:1,
			dpl			:2,
			present		:1,
			off_high	:16;
} __attribute__ ((packed)) idt_t;

/*
Two fields of that struct, off_low и off_high, contain the handler address.
	off_low - last significant bits.
	off_high - most significant bits.
*/

typedef struct
{
	__u64	limit_low	:16,
			base_low	:24,
			reserved	:8,
			type		:4,
			system		:1,
			dpl			:2,
			present		:1,
			limit_high	:4,
			avl			:1,
			o			:1,
			db			:1,
			gran		:1,
			base_high	:8;
} __attribute__ ((packed)) gdt_t;

typedef struct
{
	__u32	present		:1,
			read_write	:1,
			user		:1,
			write_thr	:1,
			cache_dis	:1,
			accessed	:1,
			reserved	:1,
			size		:1,
			global		:1,
			avl			:3,
			base		:20;
} __attribute__ ((packed)) pagedir_t;


typedef struct
{
	__u32	present		:1,
			read_write	:1,
			user		:1,
			write_thr	:1,
			cache_dis	:1,
			accessed	:1,
			dirty		:1,
			reserved	:1,
			global		:1,
			avl			:3,
			base		:20;
} __attribute__ ((packed)) pagetab_t;



static idt_t *idt;
static gdt_t *gdt;
static pagedir_t *pagedir;
static pagetab_t *pagetab;

static idt_t *idt_counter;
static gdt_t *gdt_counter;
static pagedir_t *pd_counter;
static pagetab_t *pt_counter;
static void *page;

static struct proc_dir_entry *Our_Proc_File;

// IDTR, Interrupt Descriptor Table Register:

static struct
{
	__u16 limit;		// IDT limit
	__u32 base;		// IDT base
} __attribute__ ((packed)) idtr, gdtr;

static char *cr0_flags[] = {
	"pe", "mp", "em", "ts", "et", "ne", NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	"wp", NULL, "am", NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, "nw", "cd", "pg"
};


static char *cr4_flags[] = {
	"vme", "pvi", "tsd", "de", "pse", "pae", "mce", "pge", "pce"
};


extern void ex_00 (void);
extern void ex_01 (void);
extern void ex_02 (void);
extern void ex_03 (void);
extern void ex_04 (void);
extern void ex_05 (void);
extern void ex_06 (void);
extern void ex_07 (void);
extern void ex_08 (void);
extern void ex_09 (void);
extern void ex_10 (void);
extern void ex_11 (void);
extern void ex_12 (void);
extern void ex_13 (void);
extern void ex_14 (void);
extern void ex_15 (void);
extern void ex_16 (void);
extern void ex_17 (void);
extern void ex_18 (void);
extern void ex_19 (void);
extern void ex_20 (void);
extern void ex_21 (void);
extern void ex_22 (void);
extern void ex_23 (void);
extern void ex_24 (void);
extern void ex_25 (void);
extern void ex_26 (void);
extern void ex_27 (void);
extern void ex_28 (void);
extern void ex_29 (void);
extern void ex_30 (void);
extern void ex_31 (void);


// get_handler(i) - returns the address of interrupt i handler. 


__u32
get_handler (int i)
{
	return ((idt[i].off_high << 0x10) | idt[i].off_low);
}

// set_handler() - changes one handler address in IDT.

void
set_handler (int i, __u32 new_addr)
{
	idt[i].off_high = (__u16) (new_addr >> 16);
	idt[i].off_low = (__u16) (new_addr & 0x0000FFFF);
}

void
my_exeption_00 (void)
{
	exeption_count[0]++;
}

void
my_exeption_01 (void)
{
	exeption_count[1]++;
}

void
my_exeption_02 (void)
{
	exeption_count[2]++;
}

void
my_exeption_03 (void)
{
	exeption_count[3]++;
}

void
my_exeption_04 (void)
{
	exeption_count[4]++;
}

void
my_exeption_05 (void)
{
	exeption_count[5]++;
}

void
my_exeption_06 (void)
{
	exeption_count[6]++;
}

void
my_exeption_07 (void)
{
	exeption_count[7]++;
}

void
my_exeption_08 (void)
{
	exeption_count[8]++;
}

void
my_exeption_09 (void)
{
	exeption_count[9]++;
}

void
my_exeption_10 (void)
{
	exeption_count[10]++;
}

void
my_exeption_11 (void)
{
	exeption_count[11]++;
}

void
my_exeption_12 (void)
{
	exeption_count[12]++;
}

void
my_exeption_13 (void)
{
	exeption_count[13]++;
}

void
my_exeption_14 (void)
{
	exeption_count[14]++;
}

void
my_exeption_15 (void)
{
	exeption_count[15]++;
}

void
my_exeption_16 (void)
{
	exeption_count[16]++;
}

void
my_exeption_17 (void)
{
	exeption_count[17]++;
}

void
my_exeption_18 (void)
{
	exeption_count[18]++;
}

void
my_exeption_19 (void)
{
	exeption_count[19]++;
}

void
my_exeption_20 (void)
{
	exeption_count[20]++;
}

void
my_exeption_21 (void)
{
	exeption_count[21]++;
}

void
my_exeption_22 (void)
{
	exeption_count[22]++;
}

void
my_exeption_23 (void)
{
	exeption_count[23]++;
}

void
my_exeption_24 (void)
{
	exeption_count[24]++;
}

void
my_exeption_25 (void)
{
	exeption_count[25]++;
}

void
my_exeption_26 (void)
{
	exeption_count[26]++;
}

void
my_exeption_27 (void)
{
	exeption_count[27]++;
}

void
my_exeption_28 (void)
{
	exeption_count[28]++;
}

void
my_exeption_29 (void)
{
	exeption_count[29]++;
}

void
my_exeption_30 (void)
{
	exeption_count[30]++;
}

void
my_exeption_31 (void)
{
	exeption_count[31]++;
}

//------------------------------------------------------------------------------
void
my_exeption_list_1 (void)
{
      asm (	".globl ex_00		\n"
			".globl ex_01		\n"
			".globl ex_02		\n"
			".globl ex_03		\n" ".globl ex_04		\n" ".globl ex_05		\n" ".globl ex_06		\n" ".globl ex_07		\n" ".globl ex_08		\n" ".globl ex_09		\n" ".globl ex_10		\n" ".globl ex_11		\n" ".globl ex_12		\n" ".globl ex_13		\n" ".globl ex_14		\n" "ex_00:				\n" "       cli         \n" "		call *%0	\n" "		jmp *%1		\n" "ex_01:				\n" "       cli         \n" "		call *%2	\n" "		jmp *%3		\n" "ex_02:				\n" "       cli         \n" "		call *%4	\n" "		jmp *%5		\n" "ex_03:				\n" "       cli         \n" "		call *%6	\n" "		jmp *%7		\n" "ex_04:				\n" "       cli         \n" "		call *%8	\n" "		jmp *%9		\n" "ex_05:				\n" "       cli         \n" "		call *%10	\n" "		jmp *%11	\n" "ex_06:				\n" "       cli         \n" "		call *%12	\n" "		jmp *%13	\n" "ex_07:				\n" "       cli         \n" "		call *%14	\n" "		jmp *%15	\n" "ex_08:				\n" "       cli         \n" "		call *%16	\n" "		jmp *%17	\n" "ex_09:				\n" "       cli         \n" "		call *%18	\n" "		jmp *%19	\n" "ex_10:				\n" "       cli         \n" "		call *%20	\n" "		jmp *%21	\n" "ex_11:				\n" "       cli         \n" "		call *%22	\n" "		jmp *%23	\n" "ex_12:				\n" "       cli         \n" "		call *%24	\n" "		jmp *%25	\n" "ex_13:				\n" "       cli         \n" "		call *%26	\n" "		jmp *%27	\n" "ex_14:				\n" "       cli         \n" "		call *%28	\n" "		jmp *%29	\n":
      :"m" (my_exeption_00), "m" (original_handler[0]),
"m" (my_exeption_01), "m" (original_handler[1]),
"m" (my_exeption_02), "m" (original_handler[2]),
"m" (my_exeption_03), "m" (original_handler[3]),
"m" (my_exeption_04), "m" (original_handler[4]),
"m" (my_exeption_05), "m" (original_handler[5]),
"m" (my_exeption_06), "m" (original_handler[6]),
"m" (my_exeption_07), "m" (original_handler[7]),
"m" (my_exeption_08), "m" (original_handler[8]),
"m" (my_exeption_09), "m" (original_handler[9]),
"m" (my_exeption_10), "m" (original_handler[10]),
"m" (my_exeption_11), "m" (original_handler[11]),
"m" (my_exeption_12), "m" (original_handler[12]),
"m" (my_exeption_13), "m" (original_handler[13]),
"m" (my_exeption_14), "m" (original_handler[14]));
}


void
my_exeption_list_2 (void)
{
      asm (".globl ex_15		\n" ".globl ex_16		\n" ".globl ex_17		\n" ".globl ex_18		\n" ".globl ex_19		\n" ".globl ex_20		\n" ".globl ex_21		\n" ".globl ex_22		\n" ".globl ex_23		\n" ".globl ex_24		\n" ".globl ex_25		\n" ".globl ex_26		\n" ".globl ex_27		\n" ".globl ex_28		\n" ".globl ex_29		\n" "ex_15:			\n" "       cli         \n" "		call *%0	\n" "		jmp *%1	\n" "ex_16:				\n" "       cli         \n" "		call *%2	\n" "		jmp *%3        \n" "ex_17:				\n" "       cli         \n" "		call *%4	\n" "		jmp *%5        \n" "ex_18:				\n" "       cli         \n" "		call *%6	\n" "		jmp *%7        \n" "ex_19:				\n" "       cli         \n" "		call *%8	\n" "		jmp *%9       \n" "ex_20:				\n" "       cli         \n" "		call *%10	\n" "		jmp *%11	\n" "ex_21:				\n" "       cli         \n" "		call *%12	\n" "		jmp *%13	\n" "ex_22:				\n" "       cli         \n" "		call *%14	\n" "		jmp *%15	\n" "ex_23:				\n" "       cli         \n" "		call *%16	\n" "		jmp *%17	\n" "ex_24:				\n" "       cli         \n" "		call *%18	\n" "		jmp *%19	\n" "ex_25:				\n" "       cli         \n" "		call *%20	\n" "		jmp *%21	\n" "ex_26:				\n" "       cli         \n" "		call *%22	\n" "		jmp *%23	\n" "ex_27:				\n" "       cli         \n" "		call *%24	\n" "		jmp *%25	\n" "ex_28:				\n" "       cli         \n" "		call *%26	\n" "		jmp *%27	\n" "ex_29:				\n" "       cli         \n" "		call *%28	\n" "		jmp *%29	\n":
      :"m" (my_exeption_15), "m" (original_handler[15]),
"m" (my_exeption_16), "m" (original_handler[16]),
"m" (my_exeption_17), "m" (original_handler[17]),
"m" (my_exeption_18), "m" (original_handler[18]),
"m" (my_exeption_19), "m" (original_handler[19]),
"m" (my_exeption_20), "m" (original_handler[20]),
"m" (my_exeption_21), "m" (original_handler[21]),
"m" (my_exeption_22), "m" (original_handler[22]),
"m" (my_exeption_23), "m" (original_handler[23]),
"m" (my_exeption_24), "m" (original_handler[24]),
"m" (my_exeption_25), "m" (original_handler[25]),
"m" (my_exeption_26), "m" (original_handler[26]),
"m" (my_exeption_27), "m" (original_handler[27]),
"m" (my_exeption_28), "m" (original_handler[28]),
"m" (my_exeption_29), "m" (original_handler[29]));
}



void
my_exeption_list_3 (void)
{
      asm (".globl ex_30		\n" 
	       ".globl ex_31		\n" 
	"ex_30:				\n" 
	"       cli         \n" 
	"		call *%0	\n" 
	"		jmp *%1        \n" 
	"ex_31:				\n" 
	"       cli         \n" 
	"		call *%2	\n" 
	"		jmp *%3		\n":
      :"m" (my_exeption_30), "m" (original_handler[30]),
"m" (my_exeption_31), "m" (original_handler[31]));
}

/*

*/
//------------------------------------------------------------------------------

static ssize_t
REGS_output (struct file *filp,	/* see include/linux/fs.h   */
	     char *buffer,	/* buffer to fill with data */
	     size_t length,	/* length of the buffer     */
	     loff_t * offset)
{
	static int finished = 0;
	int i;
	char message[MESSAGE_LENGTH + 30];
	char buf[100];

	/*
	 * We return 0 to indicate end of file, that we have
	 * no more information. Otherwise, processes will
	 * continue to read from us in an endless loop.
	 */
	if (finished > 0)
	{
		finished = 0;
		return 0;
	}

	/*
	 * We use put_user to copy the string from the kernel's
	 * memory segment to the memory segment of the process
	 * that called us. get_user, BTW, is
	 * used for the reverse.
	 */

	message[0] = '\0';
	// ToDo: get REGs here!

	sprintf (buf, "IDTR: base is %08x , limit is %04x\n",
		 (unsigned int) idtr.base, (unsigned short) idtr.limit);
	strcat (message, buf);
	sprintf (buf, "GDTR: base is %08x , limit is %04x\n",
		 (unsigned int) gdtr.base, (unsigned short) gdtr.limit);
	strcat (message, buf);
	sprintf (buf, "TR=%04x LDTR=%04x\n", (unsigned short) (tr),
		 (unsigned short) ldtr);
	strcat (message, buf);
	sprintf (buf,
		 "Segment registers: cs=%04x ds=%04x es=%04x fs=%04x gs=%04x ss=%04x\n",
		 (unsigned short) (cs), (unsigned short) (ds),
		 (unsigned short) (es), (unsigned short) (fs),
		 (unsigned short) (gs), (unsigned short) (fs));
	strcat (message, buf);
	sprintf (buf, "Control registers: cr0=%08x, ", (unsigned int) cr0);
	strcat (message, buf);
	for (i = 0; i < 32; i++)
	{
		if (test_bit (i, &cr0) && cr0_flags[i])
		{
			sprintf (buf, "%s ", cr0_flags[i]);
			strcat (message, buf);
		}
	}
	sprintf (buf, ", cr3 = %08x ", cr3);
	strcat (message, buf);
	if (cr3 & 0x08)
	{
		sprintf (buf, "pwt ");
		strcat (message, buf);
	}
	if (cr3 & 0x10)
	{
		sprintf (buf, "pcd ");
		strcat (message, buf);
	}
	sprintf (buf, "pgdir=%08x\n", cr3 & 0xFFFFF000);
	strcat (message, buf);
	sprintf (buf, "cr4 = 0x%08x ", cr4);
	strcat (message, buf);
	for (i = 0; i < 9; i++)
		if (test_bit (i, &cr4))
		{
			sprintf (buf, "%s ", cr4_flags[i]);
			strcat (message, buf);
		}
	sprintf (buf, "\n");
	strcat (message, buf);


	sprintf (buf,
		"dr0 = %08x dr1 = %08x dr2 = %08x dr3 = %08x dr6 = %08x dr7 = %08x\n",
		dr0, dr1, dr2, dr3, dr6, dr7);
	strcat (message, buf);

	dbnr = dr6 & DR6_DR_MASK;
	if (dbnr)
	{
		int nr;
		switch (dbnr)
		{
		case 1:
			nr = 0;
			break;
		case 2:
			nr = 1;
			break;
		case 4:
			nr = 2;
			break;
		default:
			nr = 3;
			break;
		}
		sprintf (buf, "debug register hit = %d", nr);
		strcat (message, buf);
	}
	else if (dr6 & DR_STEP)
	{
		sprintf (buf, "single step");
		strcat (message, buf);
	}
	else if (dr6 & DR_SWITCH)
	{
		sprintf (buf, "task switch");
		strcat (message, buf);
	}



	for (i = 0; i < length && message[i]; i++)
	{
		put_user (message[i], buffer + i);
	}

	/*
	 * Notice, we assume here that the size of the message
	 * is below len, or it will be received cut. In a real
	 * life situation, if the size of the message is less
	 * than len then we'd return len and on the second call
	 * start filling the buffer with the len+1'th byte of
	 * the message.
	 */
	finished++;

	return i;		/* Return the number of bytes "read" */
}

static ssize_t
module_input (struct file *filp, const char *buff, size_t len, loff_t * off)
{
	int i;
	/*
	 * Put the input into Message, where module_output
	 * will later be able to use it
	 */
	for (i = 0; i < MESSAGE_LENGTH - 1 && i < len; i++)
		get_user (Message[i], buff + i);

	Message[i] = '\0';	/* we want a standard, zero terminated string */
	return i;
}

/*
 * This function decides whether to allow an operation
 * (return zero) or not allow it (return a non-zero
 * which indicates why it is not allowed).
 *
 * The operation can be one of the following values:
 * 0 - Execute (run the "file" - meaningless in our case)
 * 2 - Write (input to the kernel module)
 * 4 - Read (output from the kernel module)
 *
 * This is the real function that checks file
 * permissions. The permissions returned by ls -l are
 * for referece only, and can be overridden here.
 */

static int
module_permission (struct inode *inode, int op, struct nameidata *foo)
{
	/*
	 * We allow everybody to read from our module, but
	 * only root (uid 0) may write to it
	 */
	if (op == 4 || (op == 2 && current->euid == 0))
		return 0;

	/*
	 * If it's anything else, access is denied
	 */
	return -EACCES;
}

/*
 * The file is opened - we don't really care about
 * that, but it does mean we need to increment the
 * module's reference count.
 */
/* int */
/* module_open (struct inode *inode, struct file *file) */
/* { */
/* 	try_module_get (THIS_MODULE); */
/* 	return seq_release(; */
/* } */

/*
 * The file is closed - again, interesting only because
 * of the reference count.
 */
int
module_close (struct inode *inode, struct file *file)
{
	module_put (THIS_MODULE);
	struct seq_file *m = (struct seq_file *) file->private_data;
	kfree (m->private);
	return seq_release (inode, file);
}




/*
 * The sequence iterator functions.  We simply use the count of the
 * next line as our internal position.
 */


static void
ct_seq_stop (struct seq_file *s, void *v)
{
	return;
}

/*
 * The show function.
 */
static int
ct_seq_show (struct seq_file *s, void *v)
{
	char *mess = s->private;
	seq_printf (s, "%s", mess);
	return 0;
}

/* HERE IS WHERE ILIA SHOULD START LOOKING!!!!! */

static int
update_IRQ_entry (char *text_b, loff_t pos)
{
	/* pos is like a for loop iterator, test_b is an output buffer,
	 * OUTPUT_BUF_SIZE bytes long. If you need more space, update
	 * its value in the begining of the file */

	// If we had what to output, return 1, otherwise, return 0 and 
	// set the buffer to an empty string
	if (pos < 32)
	{
		sprintf (text_b, "IRQ %d handled %d %d times.\n", pos,
			exeption_count[pos]);
		return 1;
	}
	else
	{
		text_b[0] = '\0';
		return 0;
	}
}

static int
update_IDT_entry (char *text_b, loff_t pos)
{
	idt_counter = idt;

	if (pos < 256)
	{
		sprintf (text_b,
			 "IDT %03i: %d base=%08x lim=%04x type=%02i sys=%i dpl=%i pre=%i\n",
			 pos, get_handler (pos), idt_counter->sel,
			 idt_counter->type, idt_counter->system,
			 idt_counter->dpl, idt_counter->present);
		idt_counter++;
		return 1;	
	}
	else
	{
		text_b[0] = '\0';
		return 0;
	}
}

static int
update_GDT_entry (char *text_b, loff_t pos)
{
	gdt_counter = gdt;
	if (pos < 256)
	{
		sprintf (text_b,
			"GDT Address %03i: %d base=%08x lim=%04x type=%02i sys=%i dpl=%i pre=%i avl=%i D/B=%i Gran=%i\n",
			pos, ((gdt[pos].base_high << 0x18) | gdt[pos].base_low),
			((gdt[pos].limit_high << 0x10) | gdt[pos].limit_low),
			gdt_counter->type,
			gdt_counter->system, gdt_counter->dpl,
			gdt_counter->present, gdt_counter->avl,
			gdt_counter->db, gdt_counter->gran);
		gdt_counter++;
		return 1;
	}
	else
	{
		text_b[0] = '\0';
		return 0;
	}	
}

static int
update_REGS_entry (char *text_b, loff_t pos)
{
/*	
	int i;
	pos = pos + sprintf (text_b + pos, "IDTR: base is %08x , limit is %04x\n",
		 (unsigned int) idtr.base, (unsigned short) idtr.limit);

	pos = pos + sprintf (text_b + pos, "GDTR: base is %08x , limit is %04x\n",
		 (unsigned int) gdtr.base, (unsigned short) gdtr.limit);
	
	pos = pos + sprintf (text_b + pos, "TR=%04x LDTR=%04x\n", (unsigned short) (tr),
		 (unsigned short) ldtr);
	
	pos = pos + sprintf (text_b + pos,
		 "Segment registers: cs=%04x ds=%04x es=%04x fs=%04x gs=%04x ss=%04x\n",
		 (unsigned short) (cs), (unsigned short) (ds),
		 (unsigned short) (es), (unsigned short) (fs),
		 (unsigned short) (gs), (unsigned short) (fs));

	pos = pos + sprintf (text_b + pos, "Control registers: cr0=%08x, ", (unsigned int) cr0);
	
	for (i = 0; i < 32; i++)
	{
		if (test_bit (i, &cr0) && cr0_flags[i])
		{
			pos = pos + sprintf (text_b + pos, "%s ", cr0_flags[i]);
		}
	}
	pos = pos + sprintf (text_b + pos, ", cr3 = %08x ", cr3);
	
	if (cr3 & 0x08)
	{
		pos = pos + sprintf (text_b + pos, "pwt ");
	}
	if (cr3 & 0x10)
	{
		pos = pos + sprintf (text_b + pos, "pcd ");
	}
	pos = pos + sprintf (text_b + pos, "pgdir=%08x\n", cr3 & 0xFFFFF000);
	pos = pos + sprintf (text_b + pos, "cr4 = 0x%08x ", cr4);
	for (i = 0; i < 9; i++)
		if (test_bit (i, &cr4))
		{
			pos = pos + sprintf (text_b + pos, "%s ", cr4_flags[i]);
		}
	pos = pos + sprintf (text_b + pos, "\n");


	pos = pos + sprintf (text_b + pos,
		 "dr0 = %08x dr1 = %08x dr2 = %08x dr3 = %08x dr6 = %08x dr7 = %08x\n",
		 dr0, dr1, dr2, dr3, dr6, dr7);

	dbnr = dr6 & DR6_DR_MASK;
	if (dbnr)
	{
		int nr;
		switch (dbnr)
		{
		case 1:
			nr = 0;
			break;
		case 2:
			nr = 1;
			break;
		case 4:
			nr = 2;
			break;
		default:
			nr = 3;
			break;
		}
		pos = pos + sprintf (text_b + pos, "debug register hit = %d", nr);
	}
	else if (dr6 & DR_STEP)
	{
		pos = pos + sprintf (text_b + pos, "single step");

	}
	else if (dr6 & DR_SWITCH)
	{
		pos = pos + sprintf (text_b + pos, "task switch");
	}
*/	
	return 0;
}


#define FORMAT_PGDIR(entry,buffer,position) \
	sprintf(buffer+position,"| %08lX |      %05lX      | %c | %s | %c | %c | %s | %s | %c | %s | %c |\n",\
			entry, 				\
			(entry >> PAGE_SHIFT), 				\
			(entry & _PAGE_GLOBAL)?'G':' ', 		\
			(entry & _PAGE_PSE)?"4M":"4K", 			\
			'0', 						\
			(entry & _PAGE_ACCESSED)?'A':' ', 		\
			(entry & _PAGE_PCD)?"CD":"  ", 			\
			(entry & _PAGE_PWT)?"WT":"WB", 			\
			(entry & _PAGE_USER)?'U':'S', 			\
			(entry & _PAGE_RW)?"RW":"RO", 			\
			(entry & _PAGE_PRESENT)?'+':'-');

static int
update_PDIR_entry (char *text_b, loff_t pos)
{
	__u32	cr3, pd_log, *pd_lin, d;
	int		len = 0;
	
	if (pos >= 1024)
	{
		pos = 1024;
		text_b[0] = '\0';
		return 0;
	}	

	asm("movl %%cr3,%0\n":"=r"(cr3));
	pd_log = cr3 & 0xFFFFF000;
	pd_lin = (unsigned long *)(pd_log + 0xC0000000);
	
	d = pd_lin[pos];

	len = len + sprintf(text_b + len,"| %04d ",  (int)pos);
	len = len + FORMAT_PGDIR(d,text_b,len);
	
	return 1;
}

static int
update_PTAB_entry (char *text_b, loff_t pos)
{
	__u32	cr3, pd_log, pt_log;
	pagedir_t	*pd_lin;
	pagetab_t	*pt_lin, d;
	int		i, len = 0;
	
	
	if (pos >= 1024)
	{
		pos = 1024;
		text_b[0] = '\0';
		return 0;
	}	

	asm("movl %%cr3,%0\n":"=r"(cr3));
	pd_log = cr3 & 0xFFFFF000;
	pd_lin = (pagedir_t *)(pd_log + 0xC0000000);
	
	while (!(pd_lin->present))
	{
		pd_lin++;
	}
	
	printk(KERN_ALERT"pd_lin = 0x%08x\n", pd_lin);
	
	pt_log = (pd_lin->base) << 0xC;
	
	printk(KERN_ALERT"pt_log = 0x%08x\n", pt_log);
	
	pt_lin = (pagetab_t *)(pt_log + 0xC0000000);
	
	printk(KERN_ALERT"pt_lin = 0x%08x\n", pt_lin);
	
	printk(KERN_ALERT"pos = %d\n", pos);	
	
	d = pt_lin[pos];
	
	printk(KERN_ALERT"d = 0x%08x\n", d);
	
	sprintf (text_b,
		"Page table entry %04d: %d present=%i base=%05x R/W=%i U/S=%i WT=%i cd=%i acc=%i dirty=%i glob=%i avl=%i\n",
		pos, d.present, d.base, d.read_write, d.user, d.write_thr, d.cache_dis,
		d.accessed,	d.dirty, d.global, d.avl);
	
	return 1;
}


/*
 * Tie them all together into a set of seq_operations.
 */
/*
 * Time to set up the file operations for our /proc file.  In this case,
 * all we need is an open function which sets up the sequence ops.
 */



#define MAKE_OPENS(name) \
static void *name##_seq_start(struct seq_file *s, loff_t *pos)\
{\
	if (!update_##name##_entry(s->private, *pos)) \
		return NULL; \
	return s->private;\
}\
\
\
static void *name##_seq_next(struct seq_file *s, void *v, loff_t *pos) \
{ \
	(*pos)++; \
	if (!update_##name##_entry(s->private, *pos)) \
		return NULL; \
	return v; \
} \
 \
 \
static struct seq_operations name##_seq_ops = { \
	.start = name##_seq_start, \
	.next  = name##_seq_next, \
	.stop  = ct_seq_stop, \
	.show  = ct_seq_show  \
}; \
 \
 \
static int name##_open(struct inode *inode, struct file *file) \
{ \
	int retval; \
	char *out_buf; \
	 \
	out_buf = kmalloc(OUTPUT_BUF_SIZE, GFP_KERNEL); \
	if (!out_buf) \
		return -ENOMEM; \
	memset(out_buf,0,OUTPUT_BUF_SIZE); \
	 \
	retval = seq_open(file, &name##_seq_ops); \
	if (retval == 0) \
		((struct seq_file *)file->private_data)->private = out_buf; \
	else \
		kfree(out_buf); \
		 \
	return retval; \
} \
\
static struct file_operations File_Ops_4_##name##_Proc_File = { \
	.write = module_input, \
	.read    = seq_read, \
	.llseek  = seq_lseek, \
	.release = seq_release, \
	.open = name##_open, \
	.release = module_close \
};


/*
 * The file is opened - we don't really care about
 * that, but it does mean we need to increment the
 * module's reference count.
 */

/*
int module_open(struct inode *inode, struct file *file)
{
  try_module_get(THIS_MODULE);
  return 0;
}

static struct file_operations File_Ops_4_REGS_Proc_File = {
  .read = REGS_output,
  .write = module_input,
  .open = module_open,
  .release = module_close,
};
*/
MAKE_OPENS (IRQ);
MAKE_OPENS (IDT);
MAKE_OPENS (GDT);
MAKE_OPENS (REGS);
MAKE_OPENS (PDIR);
MAKE_OPENS (PTAB);


/*
 * The file operations structure contains our open function along with
 * set of the canned seq_ ops.
 */

/*
 * Inode operations for our proc file. We need it so
 * we'll have some place to specify the file operations
 * structure we want to use, and the function we use for
 * permissions. It's also possible to specify functions
 * to be called for anything else which could be done to
 * an inode (although we don't bother, we just put
 * NULL).
 */

     static struct inode_operations Inode_Ops_4_Our_Proc_File = {
	     .permission = module_permission,	/* check for permissions */
     };
//------------------------------------------------------------------------------
int
init_module ()
{
	int i;

	for (i = 0; i < 256; i++)
		exeption_count[i] = 0;

	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "IRQ", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "IRQ", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_IRQ_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 1024;


	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "IDT", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "IDT", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_IDT_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 16384;


	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "GDT", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "GDT", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_GDT_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 4096;


	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "REGS", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "REGS", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_REGS_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 1024;


	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "PDIR", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "PDIR", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_PDIR_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 4096;


	Our_Proc_File =
		create_proc_entry (PROC_ENTRY_FILENAME "PTAB", 0644, NULL);
	if (Our_Proc_File == NULL)
	{
		rv = -ENOMEM;
		remove_proc_entry (PROC_ENTRY_FILENAME "PTAB", &proc_root);
		printk (KERN_INFO "Error: Could not initialize /proc/test\n");
	}

	Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
	Our_Proc_File->proc_fops = &File_Ops_4_PTAB_Proc_File;
	Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 4096;
	/* End of Proc Voodoo */
//------------------------------------------------------------------------------
      asm ("sidt %0":"=m" (idtr));
      asm ("sgdt %0":"=m" (gdtr));
      asm ("sldt %0":"=m" (ldtr));
      asm ("str %0":"=m" (tr));
      asm ("movw %%cs, %%ax\n" "movw %%ax, %0\n":"=m" (cs));
      asm ("movw %%ds, %%ax\n" "movw %%ax, %0\n":"=m" (ds));
      asm ("movw %%es, %%ax\n" "movw %%ax, %0\n":"=m" (es));
      asm ("movw %%fs, %%ax\n" "movw %%ax, %0\n":"=m" (fs));
      asm ("movw %%gs, %%ax\n" "movw %%ax, %0\n":"=m" (gs));
      asm ("movw %%ss, %%ax\n" "movw %%ax, %0\n":"=m" (ss));
      asm ("movl %%cr0,%0\n\t":"=r" (cr0));
      asm ("movl %%cr3,%0\n\t":"=r" (cr3));
      asm ("movl %%cr4,%0\n\t":"=r" (cr4));
      asm ("movl %%db0,%0\n\t":"=r" (dr0));
      asm ("movl %%db1,%0\n\t":"=r" (dr1));
      asm ("movl %%db2,%0\n\t":"=r" (dr2));
      asm ("movl %%db3,%0\n\t":"=r" (dr3));
      asm ("movl %%db6,%0\n\t":"=r" (dr6));
      asm ("movl %%db7,%0\n\t":"=r" (dr7));
//------------------------------------------------------------------------------
	printk (KERN_ALERT "IDTR: base is %08x , limit is %04x\n",
		(unsigned int) idtr.base, (unsigned short) idtr.limit);
	printk (KERN_ALERT "GDTR: base is %08x , limit is %04x\n",
		(unsigned int) gdtr.base, (unsigned short) gdtr.limit);
	printk (KERN_ALERT "TR=%04x LDTR=%04x\n", (unsigned short) (tr),
		(unsigned short) ldtr);
	printk (KERN_ALERT
		"Segment registers: cs=%04x ds=%04x es=%04x fs=%04x gs=%04x ss=%04x\n",
		(unsigned short) (cs), (unsigned short) (ds),
		(unsigned short) (es), (unsigned short) (fs),
		(unsigned short) (gs), (unsigned short) (fs));
	printk (KERN_ALERT "Control registers: cr0=%08x, ",
		(unsigned int) cr0);
	for (i = 0; i < 32; i++)
	{
		if (test_bit (i, &cr0) && cr0_flags[i])
			printk ("%s ", cr0_flags[i]);
	}
	printk (", cr3 = %08x ", cr3);
	if (cr3 & 0x08)
		printk ("pwt ");
	if (cr3 & 0x10)
		printk ("pcd ");
	printk ("pgdir=%08x\n", cr3 & 0xFFFFF000);
	printk (KERN_ALERT "cr4 = 0x%08x ", cr4);
	for (i = 0; i < 9; i++)
		if (test_bit (i, &cr4))
			printk ("%s ", cr4_flags[i]);
	printk ("\n");


	printk (KERN_ALERT
		"dr0 = %08x dr1 = %08x dr2 = %08x dr3 = %08x dr6 = %08x dr7 = %08x\n",
		dr0, dr1, dr2, dr3, dr6, dr7);

	dbnr = dr6 & DR6_DR_MASK;
	if (dbnr)
	{
		int nr;
		switch (dbnr)
		{
		case 1:
			nr = 0;
			break;
		case 2:
			nr = 1;
			break;
		case 4:
			nr = 2;
			break;
		default:
			nr = 3;
			break;
		}
		printk ("debug register hit = %d", nr);
	}
	else if (dr6 & DR_STEP)
	{
		printk ("single step");
	}
	else if (dr6 & DR_SWITCH)
	{
		printk ("task switch");
	}
	printk ("\n");

//------------------------------------------------------------------------------
	idt_addr = idtr.base;
	gdt_addr = gdtr.base;

// Allocating memory and copying the IDT, GDT, Page directory:

	idt = (idt_t *) kmalloc (256 * sizeof (idt_t), GFP_KERNEL);
	memcpy ((void *) idt, (void *) idt_addr, 256 * sizeof (idt_t));
	gdt = (gdt_t *) kmalloc (256 * sizeof (gdt_t), GFP_KERNEL);
	memcpy ((void *) gdt, (void *) gdt_addr, 256 * sizeof (gdt_t));

/*
	idt_counter = idt;
	for (i = 0; i < 16; i++)
	{
		printk (KERN_ALERT
			"IDT Address %03i: base=%08x lim=%04x type=%02i sys=%i dpl=%i pre=%i\n",
			i, get_handler (i), idt_counter->sel,
			idt_counter->type, idt_counter->system,
			idt_counter->dpl, idt_counter->present);
		idt_counter++;
	}
*/
/*
	gdt_counter = gdt;
	for (i = 0; i < 256; i++)
	{
		if ((gdt[i].base_high << 0x18) | gdt[i].base_low)
			printk (KERN_ALERT
				"GDT Address %03i: base=%08x lim=%04x type=%02i sys=%i dpl=%i pre=%i avl=%i D/B=%i Gran=%i\n",
				i, ((gdt[i].base_high << 0x18) | gdt[i].base_low),
				((gdt[i].limit_high << 0x10) | gdt[i].limit_low),
				gdt_counter->type,
				gdt_counter->system, gdt_counter->dpl,
				gdt_counter->present, gdt_counter->avl,
				gdt_counter->db, gdt_counter->gran);
		gdt_counter++;
	}
*/
//------------------------------------------------------------------------------

	pagedir = (pagedir_t *) ((cr3 & 0xFFFFF000) + 0xC0000000);

	printk (KERN_ALERT "Physical Page Directory Address: %08x\n", pagedir);

//	pd_counter = pagedir;
//	for (i = 0; i < 1024; i++)
//	{
//		if (pd_counter->present)
//			printk (KERN_ALERT
//				"Page directory entry %03i: base=%05x R/W=%i U/S=%i WT=%i acc=%i 4k=%i glob=%i avl=%i\n",
//				i, pd_counter->base, pd_counter->read_write,
//				pd_counter->user, pd_counter->accessed,
//				pd_counter->size, pd_counter->global,
//				pd_counter->avl);
//		pd_counter++;
//	}

/*
	pd_counter = pagedir;
	for (i = 1023; i > 0; i--)
	{
		if ((pd_counter->present) && !(pd_counter->size))
		{
			pagetab = (pagetab_t *)(((pd_counter->base) << 0xC) + 0xC0000000);
			printk (KERN_ALERT
				"Fetched Page table num %i: %08x\n", i,
				pagetab);
			break;
		}
		pd_counter--;
	}

	printk (KERN_ALERT "Example Page table: %08x\n", pagetab);

	pt_counter = pagetab;
	for (i = 0; i < 1024; i++)
	{
		if (pt_counter->present)
			printk (KERN_ALERT
				"Page table entry %03i: base=%05x R/W=%i U/S=%i WT=%i cd=%i acc=%i dirty=%i glob=%i avl=%i\n",
				i, pt_counter->base, pt_counter->read_write,
				pt_counter->user, pt_counter->write_thr,
				pt_counter->cache_dis, pt_counter->accessed,
				pt_counter->dirty, pt_counter->global,
				pt_counter->avl);
		pt_counter++;
	}

	pt_counter = pagetab;
	for (i = 1023; i > 0; i--)
	{
		if ((pt_counter->present))
		{
			printk (KERN_ALERT "Fetched Page num %i: %05x\n", i,
				pt_counter->base);
			break;
		}
		pt_counter--;
	}
	page = (unsigned long *) (((pt_counter->base) << 0xC) + 0xC0000000);
	printk (KERN_ALERT "Example Page: %08x\n", page);
*/
//------------------------------------------------------------------------------        


// Performing some changes in the copy of IDT.
	for (i = 0; i < 32; i++)
		original_handler[i] = get_handler (i);

	new_addr = (__u32) & ex_00;
	set_handler (0, new_addr);

	new_addr = (__u32) & ex_01;
	set_handler (1, new_addr);

	new_addr = (__u32) & ex_02;
	set_handler (2, new_addr);

	new_addr = (__u32) & ex_03;
	set_handler (3, new_addr);

	new_addr = (__u32) & ex_04;
	set_handler (4, new_addr);

	new_addr = (__u32) & ex_05;
	set_handler (5, new_addr);

	new_addr = (__u32) & ex_06;
	set_handler (6, new_addr);

	new_addr = (__u32) & ex_07;
	set_handler (7, new_addr);

	new_addr = (__u32) & ex_08;
	set_handler (8, new_addr);

	new_addr = (__u32) & ex_09;
	set_handler (9, new_addr);

	new_addr = (__u32) & ex_10;
	set_handler (10, new_addr);

	new_addr = (__u32) & ex_11;
	set_handler (11, new_addr);

	new_addr = (__u32) & ex_12;
	set_handler (12, new_addr);

	new_addr = (__u32) & ex_13;
	set_handler (13, new_addr);

	new_addr = (__u32) & ex_14;
//      printk (KERN_ALERT "The new handler address must be: %08x\n",
//              new_addr);
//      current_irq = HERE;
	set_handler (14, new_addr);
//      printk (KERN_ALERT "Original was: %08x, the new one: %08x\n",
//              original_handler[14], get_handler (HERE));


	new_addr = (__u32) & ex_15;
	set_handler (15, new_addr);

	new_addr = (__u32) & ex_16;
	set_handler (16, new_addr);

	new_addr = (__u32) & ex_17;
	set_handler (17, new_addr);

	new_addr = (__u32) & ex_18;
	set_handler (18, new_addr);

	new_addr = (__u32) & ex_19;
	set_handler (19, new_addr);

	new_addr = (__u32) & ex_20;
	set_handler (20, new_addr);

	new_addr = (__u32) & ex_21;
	set_handler (21, new_addr);

	new_addr = (__u32) & ex_22;
	set_handler (22, new_addr);

	new_addr = (__u32) & ex_23;
	set_handler (23, new_addr);

	new_addr = (__u32) & ex_24;
	set_handler (24, new_addr);

	new_addr = (__u32) & ex_25;
	set_handler (25, new_addr);

	new_addr = (__u32) & ex_26;
	set_handler (26, new_addr);

	new_addr = (__u32) & ex_27;
	set_handler (27, new_addr);

	new_addr = (__u32) & ex_28;
	set_handler (28, new_addr);

	new_addr = (__u32) & ex_29;
	set_handler (29, new_addr);

	new_addr = (__u32) & ex_30;
	set_handler (30, new_addr);

	new_addr = (__u32) & ex_31;
	set_handler (31, new_addr);


// Loading to IDTR the address of the copy:

	idtr.base = (__u32) idt;
	asm ("lidt %0"::"m" (idtr));



	return 0;
}


void
cleanup_module ()
{
//	int i;
// Restoring the original IDT address:

	idtr.base = (__u32) idt_addr;
	asm ("lidt %0"::"m" (idtr));

//	for (i=0;i<32;i++) printk (KERN_ALERT "IRQ %d handled %d times\n", i, exeption_count[i]);

// Freeing allocated memory:

	kfree (idt);
	kfree (gdt);

// Unregister the function from the proc FS:

	remove_proc_entry (PROC_ENTRY_FILENAME "IRQ", &proc_root);
	remove_proc_entry (PROC_ENTRY_FILENAME "IDT", &proc_root);
	remove_proc_entry (PROC_ENTRY_FILENAME "GDT", &proc_root);
	remove_proc_entry (PROC_ENTRY_FILENAME "REGS", &proc_root);
	remove_proc_entry (PROC_ENTRY_FILENAME "PDIR", &proc_root);
	remove_proc_entry (PROC_ENTRY_FILENAME "PTAB", &proc_root);

	printk (KERN_ALERT "my_ioctl_module is unloaded.\n");
}

MODULE_LICENSE ("GPL");
