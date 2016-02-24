#include "menable.h"
#include "menable3.h"
#include "menable4.h"

static void
dump_sgl_entry_me3(struct device *dev, struct plx_chain *plx)
{
#ifdef USE64
	dev_dbg(dev, "pci %08x %08x length 0x%x local %08x next %08x\n",
			plx->pci_address, plx->pci_high, plx->length,
			plx->local_address, plx->next);
#else
	dev_dbg(dev, "pci %08x length 0x%x local %08x next %08x\n",
			plx->pci_address, plx->length,
			plx->local_address, plx->next);
#endif
}

static void
dump_sgl_me3(struct device *dev, struct menable_dmabuf *sb)
{
	struct men_dma_chain *chain = sb->dmat;

	while (chain) {
		dump_sgl_entry_me3(dev, chain->plx);
		chain = chain->next;
	}
}

void
dump_sgl(struct siso_menable *board, struct device *dev,
		struct menable_dmabuf *sb)
{
	struct device *passdev = dev ? dev : &board->dev;

	if (board->board == 1)
		dump_sgl_me3(passdev, sb);
}
