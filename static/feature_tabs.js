var tabContent, tabButtons;

tabContent = document.getElementsByClassName("tab-content");

// Attach a listener to all the tab buttons
tabButtons = document.getElementsByClassName("tab-button");

for (let i = 0; i < tabButtons.length; i++) {
    tabButtons[i].addEventListener('click', navTabContent);
}

function navTabContent(event) {
    for (let i = 0; i < tabContent.length; i++) {
        tabContent[i].classList.add('hidden');
    }

    let currentTabContent = document.getElementById(event.target.getAttribute('data-tab-id'));

    if (currentTabContent != null) {
        currentTabContent.classList.remove('hidden');
    }
}