/*	$NetBSD: ieee80211_amrr.c,v 1.2 2007/12/11 12:40:10 lukem Exp $	*/
/*	$OpenBSD: ieee80211_amrr.c,v 1.1 2006/06/17 19:07:19 damien Exp $	*/

/*-
 * Copyright (c) 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <sys/param.h>



#include "ieee80211_amrr.h"

#define is_success(amn)	\
	((amn)->amn_retrycnt < (amn)->amn_txcnt / 10)
#define is_failure(amn)	\
	((amn)->amn_retrycnt > (amn)->amn_txcnt / 3)
#define is_enough(amn)		\
	((amn)->amn_txcnt > 10)
#define is_min_rate(rateIndex)		\
	(*(rateIndex) == 0)
#define is_max_rate(rateIndex, rateSet)		\
	(*(rateIndex) == (rateSet)->numItems - 1)
#define increase_rate(rateIndex)	\
	((*(rateIndex))++)
#define decrease_rate(rateIndex)	\
	((*(rateIndex))--)
#define reset_cnt(amn)		\
	do { (amn)->amn_txcnt = (amn)->amn_retrycnt = 0; } while (0)

void
ieee80211_amrr_node_init
(struct ieee80211_amrr *amrr,
 struct ieee80211_amrr_node *amn)
{
	amn->amn_success = 0;
	amn->amn_recovery = 0;
	amn->amn_txcnt = amn->amn_retrycnt = 0;
	amn->amn_success_threshold = amrr->amrr_min_success_threshold;
}

/*
 * Update ni->ni_txrate.
 */
void
ieee80211_amrr_choose
(struct ieee80211_amrr *amrr,
 struct ieee80211_amrr_node *amn,
 int* rateIndex,
 org_voodoo_wireless::IEEE::RateSet* rateSet)
{
	int need_change = 0;

	if (is_success(amn) && is_enough(amn)) {
		amn->amn_success++;
		if (amn->amn_success >= amn->amn_success_threshold &&
		    !is_max_rate(rateIndex, rateSet)) {
			amn->amn_recovery = 1;
			amn->amn_success = 0;
			increase_rate(rateIndex);
			need_change = 1;
		} else {
			amn->amn_recovery = 0;
		}
	} else if (is_failure(amn)) {
		amn->amn_success = 0;
		if (!is_min_rate(rateIndex)) {
			if (amn->amn_recovery) {
				amn->amn_success_threshold *= 2;
				if (amn->amn_success_threshold >
				    amrr->amrr_max_success_threshold)
					amn->amn_success_threshold =
					    amrr->amrr_max_success_threshold;
			} else {
				amn->amn_success_threshold =
				    amrr->amrr_min_success_threshold;
			}
			decrease_rate(rateIndex);
			need_change = 1;
		}
		amn->amn_recovery = 0;
	}

	if (is_enough(amn) || need_change)
		reset_cnt(amn);
}
