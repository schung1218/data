// SPDX-License-Identifier: GPL-2.0+
/*
 * This file contains a driver for the Nuvoton MA35D1 DMA engine
 * found on MA35D1
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 */

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/platform_data/dma-ma35d1.h>
#include <asm/cacheflush.h>

#include "dmaengine.h"
#include "virt-dma.h"

/* PDMA registers */
#define PDMA_OFFSET_CHAN_SIZE		0x10
#define PDMA_DSCT_CTL			0x0
#define   PDMA_OP_STOP			0x0
#define   PDMA_OP_BASIC		0x1
#define   PDMA_OP_SCATTER		0x2
#define   PDMA_TXTYPE			(1<<2)
#define   PDMA_SAFIX			(3<<8)
#define   PDMA_DAFIX			(3<<10)
#define   PDMA_TXWIDTH_1_BYTE		(0<<12)	
#define   PDMA_TXWIDTH_2_BYTES	(1<<12)
#define   PDMA_TXWIDTH_4_BYTES	(2<<12)
#define   PDMA_TXCNT(cnt)	((cnt)<<16)
#define   PDMA_GET_TXCNT(ctrl)	((ctrl>>16)&0xffff)
#define PDMA_DSCT_SA		0x004
#define PDMA_DSCT_DA		0x008
#define PDMA_DSCT_NEXT		0x00c
#define PDMA_CHCTL		0x400
#define PDMA_PAUSE		0x404
#define PDMA_SWREQ		0x408
#define PDMA_INTEN		0x418
#define PDMA_INTSTS		0x41C
#define PDMA_TDSTS		0x424
#define PDMA_TOUTEN		0x434
#define PDMA_TOUTIEN		0x438
#define PDMA_TOC		0x440
#define PDMA_CHRST		0x460
#define PDMA_TOUTPSC		0x470
#define PDMA_TOUTPSC1		0x474
#define PDMA_REQSEL		0x480

#define EN_HW_SG
#define EN_PDMA_DEBUG
#ifdef EN_PDMA_DEBUG
#define ENTRY()	pr_info("Enter...%s()\n", __func__)
#define LEAVE()	pr_info("Leave...%s()\n", __func__)
#else
#define ENTRY()
#define LEAVE()
#endif

#ifdef EN_PDMA_DEBUG
#define DMA_DEBUG(fmt, arg...) pr_info(fmt, ##arg)
#define DMA_DEBUG2(fmt, arg...) pr_info(fmt, ##arg)
#else
#define DMA_DEBUG(fmt, arg...)
#define DMA_DEBUG2(fmt, arg...)
#endif

#define MA35D1_DMA_MAX_CHANS			10
#define MA35D1_DMA_MAX_CHAN_DESCRIPTORS	32
#define MA35D1_DMA_MAX_CHAN_BYTES		0x10000


struct ma35d1_sg {
	u32 ctl;
	u32 src;
	u32 dst;
	u32 next;
};

struct ma35d1_desc {
	enum dma_transfer_direction	dma_dir;
//	dma_addr_t			dev_addr;
	unsigned int			sglen;
	unsigned int			dma_cycles;
	struct virt_dma_desc		vd;
//	uint8_t				es;
	uint32_t			ctl;
	struct ma35d1_peripheral	pcfg;
	unsigned int			sg_addr;
	struct ma35d1_sg		sg[];
};

struct ma35d1_chan {
	struct device *dev;
	struct virt_dma_chan		vc;
	void __iomem			*base;
	struct ma35d1_desc		*desc;

	struct dma_slave_config	cfg;

	bool				allocated;
	bool				error;
	int				ch_num;
	unsigned int			line_reqno;
	unsigned int			remain;
	u32				runtime_addr;
	u32				runtime_ctrl;
	u32				runtime_width;
	
};

struct ma35d1_dmadev {
	struct dma_device		ddev;
	struct ma35d1_chan		channels[MA35D1_DMA_MAX_CHANS];
	unsigned int			irq;
	int				nr_chans;
};

#if 0
struct ma35d1_filter_data {
	struct ma35d1_dmadev		*edma;
	struct of_phandle_args		*dma_spec;
};
#endif

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static inline struct ma35d1_chan *to_ma35d1_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct ma35d1_chan, vc.chan);
}

static inline struct ma35d1_desc *to_ma35d1_dma_desc(
	struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct ma35d1_desc, vd.tx);
}

static void ma35d1_dma_desc_free(struct virt_dma_desc *vd)
{
	kfree(container_of(vd, struct ma35d1_desc, vd));
}

static int ma35d1_terminate_all(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);
	u32 val;

	ENTRY();
	dev_dbg(chan2dev(chan), "%s: ch=%p\n", __func__, ch);

	spin_lock_irqsave(&ch->vc.lock, flags);

	if (ch->desc) {
		ma35d1_dma_desc_free(&ch->desc->vd);
		ch->desc = NULL;
	}

	val = readl(ch->base + PDMA_CHCTL) & ~(1 << ch->ch_num);
	writel(val, ch->base + PDMA_CHCTL);

	vchan_get_all_descriptors(&ch->vc, &head);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	vchan_dma_desc_free_list(&ch->vc, &head);

	return 0;
}

static int ma35d1_slave_config(struct dma_chan *chan,
			       struct dma_slave_config *cfg)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
#if 0		
	memcpy(&ch->cfg, cfg, sizeof(*cfg));
	return 0;
#else
	enum dma_slave_buswidth width;
	u32 addr, ctrl;

	ENTRY();
	ch->cfg = *cfg;

	if (ch->cfg.direction == DMA_MEM_TO_DEV) {
		ctrl = PDMA_DAFIX | PDMA_TXTYPE;
		width = ch->cfg.dst_addr_width;
		addr = ch->cfg.dst_addr;
	} else {
		ctrl = PDMA_SAFIX | PDMA_TXTYPE;
		width = ch->cfg.src_addr_width;
		addr = ch->cfg.src_addr;
	}

	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl |= PDMA_TXWIDTH_1_BYTE;
		ch->runtime_width = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl |= PDMA_TXWIDTH_2_BYTES;
		ch->runtime_width = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl |= PDMA_TXWIDTH_4_BYTES;
		ch->runtime_width = 4;
		break;
	default:
		return -EINVAL;
	}
	ch->runtime_addr = addr;
	ch->runtime_ctrl = ctrl;
	return 0;
#endif
}

static struct dma_async_tx_descriptor *
ma35d1_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dst,
		       dma_addr_t src, size_t len, unsigned long tx_flags)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct ma35d1_desc *d;
	unsigned int sg_len;
	size_t bytes, tlen;
	int i;
	
	ENTRY();
	sg_len = 1 + len / MA35D1_DMA_MAX_CHAN_BYTES;
	
	d = kzalloc(struct_size(d, sg, max_t(size_t, sg_len, 4)), GFP_ATOMIC);
	if (!d)
		return NULL;

	d->sg_addr = dma_map_single(ch->dev, d->sg, sizeof(*d->sg), DMA_TO_DEVICE);
	for (i = 0; i < sg_len; i++) {
		bytes = min_t(size_t, len, MA35D1_DMA_MAX_CHAN_BYTES);
		d->sg[i].src = src;
		d->sg[i].dst = dst;
		tlen = rounddown(len, bytes);
		d->sg[i].ctl = PDMA_TXCNT(tlen - 1UL) | PDMA_OP_STOP;
		src += tlen;
		dst += tlen;
		len -= tlen;
		d->sg[i].next = d->sg_addr+(16*(i+1)+4);
	}
	dma_sync_single_for_cpu(ch->dev, d->sg_addr, sizeof(*d->sg), DMA_FROM_DEVICE);
	d->sglen = sg_len;
	d->dma_dir = DMA_MEM_TO_MEM;
	d->pcfg.reqsel = 0;	
	return vchan_tx_prep(&ch->vc, &d->vd, tx_flags);
}

static struct dma_async_tx_descriptor *ma35d1_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction dir,
	unsigned long tx_flags, void *context)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct ma35d1_desc *d;
	//enum ddev_buswidth dev_width;
	//dma_addr_t dev_addr;
	struct scatterlist *sgent;
	//unsigned int es;
	unsigned int i;

	ENTRY();
	if (!is_slave_direction(dir)) {
		dev_err(chan2dev(chan), "%s: invalid DMA direction\n",
			__func__);
		return NULL;
	}


	d = kzalloc(struct_size(d, sg, sg_len), GFP_ATOMIC);
	if (!d)
		return NULL;


	d->sg_addr = (u32)dma_map_single(ch->dev,
			(void *)(d->sg), sizeof(*d->sg), DMA_BIDIRECTIONAL);
	//d->dma_dir = dir;
	//d->dev_addr = dev_addr;
	//d->es = es;
	for_each_sg(sgl, sgent, sg_len, i) {
		//d->sg[i].addr = sg_dma_address(sgent);
		//d->sg[i].len = sg_dma_len(sgent);
		printk("sg_dma_len(sgent) %d, runtime_width %d\n",sg_dma_len(sgent),ch->runtime_width);
		d->sg[i].ctl = (ch->runtime_ctrl | PDMA_TXCNT((sg_dma_len(sgent)/ch->runtime_width) - 1UL) | PDMA_OP_STOP);
		if (dir == DMA_MEM_TO_DEV) {
			d->sg[i].src = sg_dma_address(sgent);
			d->sg[i].dst = ch->runtime_addr;
			d->dma_dir = DMA_MEM_TO_DEV;

		} else {
			d->sg[i].src = ch->runtime_addr;
			d->sg[i].dst = sg_dma_address(sgent);
			d->dma_dir = DMA_DEV_TO_MEM;
		}
		d->sg[i].next = d->sg_addr+(PDMA_OFFSET_CHAN_SIZE*(i+1)+4);
	}
	d->sglen = sg_len;
	
	dma_sync_single_for_cpu(ch->dev, d->sg_addr, sizeof(*d->sg), DMA_FROM_DEVICE);
	if (ch->cfg.peripheral_size != sizeof(struct ma35d1_peripheral)) {
		dev_err(chan2dev(chan), "Invalid peripheral size %zu, expected %zu\n",
  			ch->cfg.peripheral_size,
			sizeof(struct ma35d1_peripheral));
		return NULL;
	}
	memcpy(&d->pcfg, ch->cfg.peripheral_config, ch->cfg.peripheral_size);

	ch->error = 0;
	printk("==========sg_len %d\n", sg_len);
	LEAVE();
	return vchan_tx_prep(&ch->vc, &d->vd, tx_flags);
}

static struct dma_chan *ma35d1_of_xlate(struct of_phandle_args *dma_spec,
					struct of_dma *of)
{
#if 1
	struct ma35d1_dmadev *edma = of->of_dma_data;
	unsigned int request;
//	struct ma35d1_chan *ch;

	if (dma_spec->args_count != 1)
		return NULL;

	request = dma_spec->args[0];
	if (request >= edma->nr_chans)
		return NULL;

	printk("=======> request %d\n",request);
//	ch->line_reqno = dma_spec->args[0];
	return dma_get_slave_channel(&(edma->channels[request].vc.chan));
#else
	struct ma35d1_dmadev *edma = of->of_dma_data;
	struct dma_chan *chan;
	struct ma35d1_chan *ch;

	chan = dma_get_any_slave_channel(&edma->ddev);
	if (!chan)
		return NULL;

	ch = to_ma35d1_dma_chan(chan);
	ch->line_reqno = dma_spec->args[0];

	printk("=======> ch->line_reqno %d\n",ch->line_reqno);
	return chan;
#endif
}

static int ma35d1_alloc_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	ENTRY();
	dev_dbg(chan2dev(chan), "%s: allocating channel #%u\n",
		__func__, ch->ch_num);
		
	printk("%s: allocating channel #%u\n",
		__func__, ch->ch_num);
	ch->allocated = 1;
	LEAVE();
	return 0;
}

static void ma35d1_free_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);

	vchan_free_chan_resources(&ch->vc);
	ENTRY();
	dev_dbg(chan2dev(chan), "%s: freeing channel #%u\n",
		__func__, ch->ch_num);
	ch->allocated = 0;
	LEAVE();
}

#if 0
static void ma35d1_dma_set_params(struct ma35d1_chan *ch, dma_addr_t src_addr,
				  dma_addr_t dst_addr)
{
	writel(src_addr, ch->base + REG_OFF_ADDRESS_SOURCE);
	writel(dst_addr, ch->base + REG_OFF_ADDRESS_DEST);
}
#endif
#if 0
static void ma35d1_set_transfer_params(struct ma35d1_chan *ch, unsigned int len)
{
	struct ma35d1_desc *d = ch->desc;
	unsigned int sglen_div = es_bytes[d->es];

	d->dma_cycles = len >> sglen_div;

	/*
	 * There are 4 cycles on 64 bytes copied, i.e. one cycle copies 16
	 * bytes ( when width is APB_DMAB_DATA_WIDTH_4 ).
	 */
	writel(d->dma_cycles, ch->base + REG_OFF_CYCLES);

	dev_dbg(chan2dev(&ch->vc.chan), "%s: set %u DMA cycles (len=%u)\n",
		__func__, d->dma_cycles, len);
}
#else

static void ma35d1_set_dma_timeout(struct ma35d1_chan *ch)
{
	struct ma35d1_desc *d = ch->desc;
	u32 val;

	ENTRY();

	if (d->pcfg.timeout_prescaler == 0 && d->pcfg.timeout_counter == 0) {
		/* Disable time-out funciton */
		val = readl(ch->base + PDMA_TOUTIEN);
		val &= ~(1<<ch->ch_num);
		writel(val, ch->base + PDMA_TOUTIEN);

		val = readl(ch->base + PDMA_TOUTEN);
		val &= ~(1<<ch->ch_num);
		writel(val, ch->base + PDMA_TOUTEN);
		return;
	}

	if (ch->ch_num <= 7) {
		val = readl(ch->base +PDMA_TOUTPSC);
		val &= ~(0x7 << (4 * ch->ch_num));
		writel(val, ch->base + PDMA_TOUTPSC);

		val = readl(ch->base + PDMA_TOUTPSC);
		val |= ((d->pcfg.timeout_prescaler & 0x7) << (4 * ch->ch_num));
		writel(val, ch->base +PDMA_TOUTPSC);

	} else {
		val = readl(ch->base + PDMA_TOUTPSC1);
		val &= ~(0x7 << (4 * (ch->ch_num - 8)));
		writel(val, ch->base + PDMA_TOUTPSC1);

		val = readl(ch->base + PDMA_TOUTPSC1);
		val |= ((d->pcfg.timeout_counter & 0x7) << (4 * (ch->ch_num - 8)));
		writel(val, ch->base + PDMA_TOUTPSC1);
	}

	val = readl(ch->base + (4 * (ch->ch_num / 2)));
	val &= ~(0xffff << (16 * (ch->ch_num % 2)));
	writel(val, ch->base + PDMA_TOC + (4 * (ch->ch_num / 2)));

	val = readl(ch->base +(4 * (ch->ch_num/ 2)));
	val |= ((d->pcfg.timeout_prescaler & 0xffff) << (16 * (ch->ch_num % 2)));
	writel(val, ch->base +PDMA_TOC + (4 * (ch->ch_num / 2)));

	/* Enable time-out funciton */
	val = readl(ch->base + PDMA_TOUTEN);
	val |= (1 << ch->ch_num);
	writel(val, ch->base + PDMA_TOUTEN);

	/* Enable time-out interrupt */
	val = readl(ch->base + PDMA_TOUTIEN);
	val |=  (1 << ch->ch_num);
	writel(val, ch->base + PDMA_TOUTIEN);
	LEAVE();
}

static void ma35d1_set_transfer_params(struct ma35d1_chan *ch)
{
	struct ma35d1_desc *d = ch->desc;
	u32 sel, reg;

	reg = PDMA_REQSEL + (4 * (ch->ch_num / 4));
	sel = readl(ch->base + reg);
	sel &= ~(0xFF << ((ch->ch_num & 0x3) * 8));
	sel |= (d->pcfg.reqsel << ((ch->ch_num & 0x3) * 8));
	writel(sel, ch->base + reg);
}
#endif

static void ma35d1_start_dma(struct ma35d1_chan *ch)
{
	struct ma35d1_desc *d = ch->desc;
	u32 ctrl, reg, val;

	ENTRY();
	if ((readl(ch->base + ch->ch_num * 16) & 0x3) != 0) {
		reg = readl(ch->base + PDMA_CHCTL);
		writel(0, ch->base + ch->ch_num * 16);
		val = readl(ch->base + PDMA_CHRST) | 1<<(ch->ch_num);
		writel(val, ch->base + PDMA_CHRST);
		writel(reg | (1<<ch->ch_num), ch->base + PDMA_CHCTL);

	} else {
		writel(0, ch->base + ch->ch_num * 16);
		val = readl(ch->base + PDMA_CHCTL) | (1 << ch->ch_num);
		writel(val, ch->base + PDMA_CHCTL);
	}
	val = readl(ch->base + PDMA_INTEN) | (1 << ch->ch_num);
	writel(val, ch->base + PDMA_INTEN);

	writel(d->sg_addr, ch->base + ch->ch_num * PDMA_OFFSET_CHAN_SIZE + PDMA_DSCT_NEXT);
	DMA_DEBUG2("d->sglen %d d->pcfg.reqsel %d ch->ch_num %d d->sg[0].ctl 0x%08x\n",d->sglen,d->pcfg.reqsel, ch->ch_num,d->sg[0].ctl);
	if (d->sglen == 1) {
		writel(d->sg[0].ctl, ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) + PDMA_DSCT_CTL);
		writel(d->sg[0].src, ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) + PDMA_DSCT_SA);
		writel(d->sg[0].dst, ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) + PDMA_DSCT_DA);

		ctrl = readl(ch->base + (PDMA_OFFSET_CHAN_SIZE * ch->ch_num));
		ctrl |= PDMA_OP_BASIC;
		writel(ctrl, ch->base + PDMA_OFFSET_CHAN_SIZE * ch->ch_num);
		if (d->pcfg.reqsel == 0)
			writel( (1<<ch->ch_num), ch->base + PDMA_SWREQ);
	} else {
		ctrl = readl(ch->base + PDMA_OFFSET_CHAN_SIZE * ch->ch_num);		
		ctrl |= PDMA_OP_SCATTER;
		writel(ctrl, ch->base + PDMA_OFFSET_CHAN_SIZE * ch->ch_num);
	}
	
	
	
	DMA_DEBUG2("===============pdma=============\n");
	DMA_DEBUG2("pdma->DSCT[%d].CTL=0x%08x\n", ch->ch_num,
		   readl(ch->base +ch->ch_num * 16));
	DMA_DEBUG2("pdma->DSCT[%d].SA=0x%08x\n", ch->ch_num,
		   readl(ch->base + (ch->ch_num * 16) + 4));
	DMA_DEBUG2("pdma->DSCT[%d].DA=0x%08x\n", ch->ch_num,
		   readl(ch->base + (ch->ch_num * 16) + 8));
	DMA_DEBUG2("pdma->CHCTL=0x%08x\n",
		   readl(ch->base + PDMA_CHCTL));
	DMA_DEBUG2("pdma->INTEN=0x%08x\n",
		   readl(ch->base +PDMA_INTEN));
	DMA_DEBUG2("pdma->INTSTS=0x%08x\n",
		   readl(ch->base + PDMA_INTSTS));
	DMA_DEBUG2("pdma->TDSTS=0x%08x\n",
		   readl(ch->base + PDMA_TDSTS));
	DMA_DEBUG2("pdma->REQSEL=0x%08x\n",
		   readl(ch->base + PDMA_REQSEL));
	DMA_DEBUG2("===============================\n");
	LEAVE();
}

static void ma35d1_dma_start_sg(struct ma35d1_chan *ch, unsigned int idx)
{
	ENTRY();
	ma35d1_set_dma_timeout(ch);
	ma35d1_set_transfer_params(ch);
	ma35d1_start_dma(ch);
	LEAVE();
}

static void ma35d1_dma_start_desc(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct virt_dma_desc *vd;
	ENTRY();
	vd = vchan_next_desc(&ch->vc);

	if (!vd) {
		ch->desc = NULL;
		return;
	}

	list_del(&vd->node);

	ch->desc = to_ma35d1_dma_desc(&vd->tx);
	//ch->sgidx = 0;

	ma35d1_dma_start_sg(ch, 0);
	LEAVE();
}

static void ma35d1_issue_pending(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	unsigned long flags;

	ENTRY();
	spin_lock_irqsave(&ch->vc.lock, flags);
	if(ch->desc==NULL)
		printk("ch->desc==NULL\n");
	if (vchan_issue_pending(&ch->vc) && !ch->desc)
		ma35d1_dma_start_desc(chan);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	LEAVE();
}

#if 0
static size_t ma35d1_dma_desc_size(struct ma35d1_desc *d,
				   unsigned int completed_sgs)
{
	unsigned int i;
	size_t size;

	for (size = i = completed_sgs; i < d->sglen; i++)
		size += PDMA_GET_TXCNT(d->sg[i].ctl);

	return size;
}

static size_t ma35d1_dma_desc_size_in_remain(struct ma35d1_chan *ch)
{
	size_t size;

	size = ma35d1_dma_desc_size(ch->desc, ch->sgidx);
	size = ch->remain + size;

	dev_dbg(chan2dev(&ch->vc.chan), "%s: size=%zu\n", __func__, size);

	return size;
}
#endif
static enum dma_status ma35d1_tx_status(struct dma_chan *chan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{

#if 1
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct virt_dma_desc *vd;
        enum dma_status ret;
        unsigned long flags;

	printk("===>1\n");
	ret = dma_cookie_status(chan, cookie, txstate);
	printk("===>2\n");
	spin_lock_irqsave(&ch->vc.lock, flags);
	if (likely(ret != DMA_ERROR)) {
	printk("===>3\n");
		dma_set_residue(txstate, ch->remain);
		printk("===>4\n");
		}
	
	//vd = vchan_find_desc(&ch->vc, cookie);
	//if (vd) 
	//	list_del(&vd->node);
	DMA_DEBUG2("==========>%s 2 txstate->residue %d\n", __func__, txstate->residue);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	printk("===>5\n");
	return ret;
#else
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct virt_dma_desc *vd;
	struct ma35d1_desc *d;
	enum dma_status ret;
	unsigned long flags;

	ENTRY();
	/*
	 * dma_cookie_status() assigns initial residue value.
	 */
	ret = dma_cookie_status(chan, cookie, txstate);

	spin_lock_irqsave(&ch->vc.lock, flags);
	//vd = vchan_find_desc(&ch->vc, cookie);
	//if (vd) {
		
	//	d = to_ma35d1_dma_desc(&vd->tx);
	//	txstate->residue = ma35d1_dma_desc_size(d, 0);
	//	DMA_DEBUG2("==========>%s 1 txstate->residue %d\n", __func__, txstate->residue);
	//} else 
	if (ch->desc && ch->desc->vd.tx.cookie == cookie) {
		txstate->residue = ma35d1_dma_desc_size_in_remain(ch);
		DMA_DEBUG2("==========>%s 2 txstate->residue %d\n", __func__, txstate->residue);
	}
	spin_unlock_irqrestore(&ch->vc.lock, flags);

	if (ch->error)
		return DMA_ERROR;

	return ret;
#endif
}
#if 0
static void ma35d1_dma_init(struct dma_device *dma, struct device *dev)
{
	dma->device_prep_slave_sg		= ma35d1_prep_slave_sg;
	dma->device_alloc_chan_resources	= ma35d1_alloc_chan_resources;
	dma->device_free_chan_resources		= ma35d1_free_chan_resources;
	dma->device_issue_pending		= ma35d1_issue_pending;
	dma->device_tx_status			= ma35d1_tx_status;
	dma->device_config			= ma35d1_slave_config;
	dma->device_terminate_all		= ma35d1_terminate_all;
	dma->dev				= dev;

	
}
#endif
static irqreturn_t ma35d1_dma_interrupt(int irq, void *devid)
{
	int i;
	u32 val;
	struct ma35d1_dmadev *dmadev = devid;
	struct ma35d1_chan *ch = &dmadev->channels[dmadev->nr_chans - 1];
	unsigned int intsts = readl(ch->base + PDMA_INTSTS);
	unsigned int tdsts =  readl(ch->base + PDMA_TDSTS);

	printk("intst 0x%08x, tdsts 0x%08x\n",intsts, tdsts);
	for (i = (dmadev->nr_chans - 1); i >= 0; i--, ch--) {
		/* Transfer done interrupt */
		if (tdsts & (1<<i)) {
			writel((1<<i), ch->base + PDMA_TDSTS);
			if (ch->desc) {
				ch->remain = 0;
				spin_lock(&ch->vc.lock);
				vchan_cookie_complete(&ch->desc->vd);
				ma35d1_dma_start_desc(&ch->vc.chan);
				spin_unlock(&ch->vc.lock);
			}
		}

		/* Timeout interrupt */
		if (intsts & (1 << (i + 8))) {
			spin_lock(&ch->vc.lock);
			val = readl(ch->base + ch->ch_num * PDMA_OFFSET_CHAN_SIZE);		
			ch->remain = PDMA_GET_TXCNT(val);
			printk("ch->desc->remain %d\n", ch->remain);
			val = readl(ch->base + PDMA_TOUTEN);
			val &= ~(1<<i);
			writel(val, ch->base + PDMA_TOUTEN);
			writel(1<<(i+8), ch->base + PDMA_INTSTS);
			vchan_cookie_complete(&ch->desc->vd);
			ma35d1_dma_start_desc(&ch->vc.chan);
			spin_unlock(&ch->vc.lock);
		}

	}
	return IRQ_HANDLED;
}

static int ma35d1_probe(struct platform_device *pdev)
{
	struct clk *pdma_clk;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;
	void __iomem *dma_base_addr;
	int ret, i, nr_chans;
	unsigned int irq;
	struct ma35d1_chan *ch;
	struct ma35d1_dmadev *edma;

	printk("=============================================>%s\n",__func__);
	pdma_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(pdma_clk)) {
		ret = PTR_ERR(pdma_clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", ret);
		return -ENOENT;
	}
	ret = clk_prepare_enable(pdma_clk);
	if (ret)
		return -ENOENT;
		
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	edma = devm_kzalloc(dev, sizeof(*edma), GFP_KERNEL);
	if (!edma)
		return -ENOMEM;

	//irq = irq_of_parse_and_map(node, 0);
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(dev, "no IRQ resource\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "dma-channels", &nr_chans))
		return -EINVAL;
	if (nr_chans > MA35D1_DMA_MAX_CHANS)
		nr_chans = MA35D1_DMA_MAX_CHANS;
	edma->nr_chans = nr_chans;

	printk("edma->nr_chans %d\n",edma->nr_chans);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dma_base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(dma_base_addr))
		return PTR_ERR(dma_base_addr);

	dma_cap_zero(edma->ddev.cap_mask);
	dma_cap_set(DMA_SLAVE, edma->ddev.cap_mask);
	dma_cap_set(DMA_PRIVATE, edma->ddev.cap_mask);
//	dma_cap_set(DMA_CYCLIC, edma->ddev.cap_mask);
	dma_cap_set(DMA_MEMCPY, edma->ddev.cap_mask);
	edma->ddev.device_prep_dma_memcpy	= ma35d1_prep_dma_memcpy;
	edma->ddev.device_prep_slave_sg		= ma35d1_prep_slave_sg;
//	edma->ddev.device_prep_dma_cyclic	= ma35d1_dma_prep_dma_cyclic;
	edma->ddev.device_alloc_chan_resources	= ma35d1_alloc_chan_resources;
	edma->ddev.device_free_chan_resources	= ma35d1_free_chan_resources;
	edma->ddev.device_issue_pending		= ma35d1_issue_pending;
	edma->ddev.device_tx_status		= ma35d1_tx_status;
	edma->ddev.device_config		= ma35d1_slave_config;
	edma->ddev.device_terminate_all		= ma35d1_terminate_all;
	edma->ddev.dev				= dev;
	edma->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	edma->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	edma->ddev.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
		BIT(DMA_MEM_TO_MEM);

	INIT_LIST_HEAD(&edma->ddev.channels);

	ch = &edma->channels[0];
	for (i = 0; i < nr_chans; i++, ch++) {
		ch->dev = &pdev->dev;
		ch->ch_num = i;
//		ch->base = dma_base_addr + i * PDMA_OFFSET_CHAN_SIZE;
		ch->base = dma_base_addr;
		ch->allocated = 0;

		ch->vc.desc_free = ma35d1_dma_desc_free;
		vchan_init(&ch->vc, &edma->ddev);
		dev_dbg(dev, "%s: chs[%d]: ch->ch_num=%u ch->base=%p\n",
			__func__, i, ch->ch_num, ch->base);
	}

	platform_set_drvdata(pdev, edma);

	ret = devm_request_irq(dev, irq, ma35d1_dma_interrupt, 0,
			       pdev->name, edma);
	if (ret) {
		dev_err(dev, "devm_request_irq failed\n");
		return ret;
	}
	edma->irq = irq;

	ret = dma_async_device_register(&edma->ddev);
	if (ret) {
		dev_err(dev, "dma_async_device_register failed\n");
		return ret;
	}

	ret = of_dma_controller_register(node, ma35d1_of_xlate, edma);
	if (ret) {
		dev_err(dev, "of_dma_controller_register failed\n");
		dma_async_device_unregister(&edma->ddev);
		return ret;
	}

	dev_dbg(dev, "%s: IRQ=%u\n", __func__, irq);
	printk("%s: IRQ=%u\n", __func__, irq);
	LEAVE();
	return 0;
}

static int ma35d1_remove(struct platform_device *pdev)
{
	struct ma35d1_dmadev *m = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, m->irq, m);

	dma_async_device_unregister(&m->ddev);

	if (pdev->dev.of_node)
		of_dma_controller_free(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id ma35d1_dma_match[] = {
	{ .compatible = "nuvoton,ma35d1-pdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, ma35d1_dma_match);

static struct platform_driver ma35d1_driver = {
	.probe	= ma35d1_probe,
	.remove	= ma35d1_remove,
	.driver = {
		.name		= "ma35d1-pdma",
		.of_match_table	= ma35d1_dma_match,
	},
};

static int ma35d1_init(void)
{
	return platform_driver_register(&ma35d1_driver);
}
subsys_initcall(ma35d1_init);

static void __exit ma35d1_exit(void)
{
	platform_driver_unregister(&ma35d1_driver);
}
module_exit(ma35d1_exit);

MODULE_AUTHOR("Shan-Chun Hung <schung@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton MA35D1 DMA controller driver");
MODULE_LICENSE("GPL v2");
