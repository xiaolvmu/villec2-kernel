#ifndef _LINUX_NLS_H
#define _LINUX_NLS_H

#include <linux/init.h>


typedef u16 wchar_t;
#define MAX_WCHAR_T	0xffff

typedef u32 unicode_t;

struct nls_table {
	const char *charset;
	const char *alias;
	int (*uni2char) (wchar_t uni, unsigned char *out, int boundlen);
	int (*char2uni) (const unsigned char *rawstring, int boundlen,
			 wchar_t *uni);
	const unsigned char *charset2lower;
	const unsigned char *charset2upper;
	struct module *owner;
	struct nls_table *next;
};

#define NLS_MAX_CHARSET_SIZE 6 

enum utf16_endian {
	UTF16_HOST_ENDIAN,
	UTF16_LITTLE_ENDIAN,
	UTF16_BIG_ENDIAN
};

extern int register_nls(struct nls_table *);
extern int unregister_nls(struct nls_table *);
extern struct nls_table *load_nls(char *);
extern void unload_nls(struct nls_table *);
extern struct nls_table *load_nls_default(void);

extern int utf8_to_utf32(const u8 *s, int len, unicode_t *pu);
extern int utf32_to_utf8(unicode_t u, u8 *s, int maxlen);
extern int utf8s_to_utf16s(const u8 *s, int len,
		enum utf16_endian endian, wchar_t *pwcs, int maxlen);
extern int utf16s_to_utf8s(const wchar_t *pwcs, int len,
		enum utf16_endian endian, u8 *s, int maxlen);

static inline unsigned char nls_tolower(struct nls_table *t, unsigned char c)
{
	unsigned char nc = t->charset2lower[c];

	return nc ? nc : c;
}

static inline unsigned char nls_toupper(struct nls_table *t, unsigned char c)
{
	unsigned char nc = t->charset2upper[c];

	return nc ? nc : c;
}

static inline int nls_strnicmp(struct nls_table *t, const unsigned char *s1,
		const unsigned char *s2, int len)
{
	while (len--) {
		if (nls_tolower(t, *s1++) != nls_tolower(t, *s2++))
			return 1;
	}

	return 0;
}

static inline int
nls_nullsize(const struct nls_table *codepage)
{
	int charlen;
	char tmp[NLS_MAX_CHARSET_SIZE];

	charlen = codepage->uni2char(0, tmp, NLS_MAX_CHARSET_SIZE);

	return charlen > 0 ? charlen : 1;
}

#define MODULE_ALIAS_NLS(name)	MODULE_ALIAS("nls_" __stringify(name))

#endif 

