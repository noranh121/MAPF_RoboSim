
// Lock scrolling
const lockScroll = () => {
    document.body.style.overflow = 'hidden';
};

// Unlock scrolling
const unlockScroll = () => {
    document.body.style.overflow = 'auto';
};

// Initial scroll lock
unlockScroll();

// Attach click event to the button
document.getElementById('scrollButton').addEventListener('click', () => {
    // unlockScroll();
    // const mainContent = document.querySelector(".mainContent");
    // mainContent.scrollIntoView({ behavior: "smooth" });
    // document.querySelector('.hero-section').scrollIntoView({ behavior: 'smooth' }); 
    
});

// Function to toggle form inputs based on the selected option
function toggleFormInputs() {
    const selectAlgo = document.getElementById("selectAlgo");
    const formFile = document.getElementById("formFile");
    const algoName = document.getElementById("algoName");
    const button = document.getElementById("upload-algo-btn");

    if (document.getElementById("selectAlgoOption").checked) {
      algoName.disabled = true;
      selectAlgo.disabled = false;
      formFile.disabled = true;
      button.disabled = true;
    } else {
      algoName.disabled = false;
      selectAlgo.disabled = true;
      formFile.disabled = false;
      button.disabled = false;
    }
}

toggleFormInputs();

//change into selected
document.getElementById("selectButton").addEventListener('click', () => {
    document.getElementById("selectButton").innerHTML = "selected";
})

let selectedMap = '';

    function selectMap(mapName, index) {
        // Reset all buttons to 'Select'
        const buttons = document.querySelectorAll('.select-button');
        buttons.forEach(button => button.innerHTML = 'Select');

        // Set the clicked button to 'Selected'
        document.getElementById('select-button-' + index).innerHTML = 'Selected';
        selectedMap = mapName;
    }

    // // Submit the selected map along with other form data
    // function submitForm() {
    //     const form = document.getElementById('simulation-form');
    //     const mapInput = document.getElementById('selected-map');
    //     mapInput.value = selectedMap;
    //     form.submit();
    // }