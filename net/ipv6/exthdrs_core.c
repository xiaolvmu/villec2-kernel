#include <linux/export.h>
#include <net/ipv6.h>


int ipv6_ext_hdr(u8 nexthdr)
{
	return   (nexthdr == NEXTHDR_HOP)	||
		 (nexthdr == NEXTHDR_ROUTING)	||
		 (nexthdr == NEXTHDR_FRAGMENT)	||
		 (nexthdr == NEXTHDR_AUTH)	||
		 (nexthdr == NEXTHDR_NONE)	||
		 (nexthdr == NEXTHDR_DEST);
}


int ipv6_skip_exthdr(const struct sk_buff *skb, int start, u8 *nexthdrp,
		     __be16 *frag_offp)
{
	u8 nexthdr = *nexthdrp;

	*frag_offp = 0;

	while (ipv6_ext_hdr(nexthdr)) {
		struct ipv6_opt_hdr _hdr, *hp;
		int hdrlen;

		if (nexthdr == NEXTHDR_NONE)
			return -1;
		hp = skb_header_pointer(skb, start, sizeof(_hdr), &_hdr);
		if (hp == NULL)
			return -1;
		if (nexthdr == NEXTHDR_FRAGMENT) {
			__be16 _frag_off, *fp;
			fp = skb_header_pointer(skb,
						start+offsetof(struct frag_hdr,
							       frag_off),
						sizeof(_frag_off),
						&_frag_off);
			if (fp == NULL)
				return -1;

			*frag_offp = *fp;
			if (ntohs(*frag_offp) & ~0x7)
				break;
			hdrlen = 8;
		} else if (nexthdr == NEXTHDR_AUTH)
			hdrlen = (hp->hdrlen+2)<<2;
		else
			hdrlen = ipv6_optlen(hp);

		nexthdr = hp->nexthdr;
		start += hdrlen;
	}

	*nexthdrp = nexthdr;
	return start;
}

EXPORT_SYMBOL(ipv6_ext_hdr);
EXPORT_SYMBOL(ipv6_skip_exthdr);
