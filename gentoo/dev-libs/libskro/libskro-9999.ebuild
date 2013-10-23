# Copyright 1999-2013 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2
# $Header: $

EAPI=4
inherit eutils git-2

DESCRIPTION="Skeletal Robot library."
HOMEPAGE="https://github.com/IntelligentRoboticsLab/KukaYouBot"
SRC_URI=""

EGIT_REPO_URI="https://github.com/IntelligentRoboticsLab/KukaYouBot.git"

LICENSE="GPL-3"
SLOT="0"
KEYWORDS="~amd64"
IUSE=""

DEPEND="dev-vcs/git"
RDEPEND="${DEPEND}"

src_unpack() {
	git-2_src_unpack
}

src_prepare() {
	cd libskro
	./autogen.sh
}

src_configure() {
	cd libskro
	econf
}

src_install() {
	cd libskro
	emake DESTDIR="${D}" install
}

