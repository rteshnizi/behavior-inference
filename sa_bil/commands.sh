function pip3-install-save {
	pip3 install $1 && pip3 freeze | grep $1 >> requirements.txt
}
