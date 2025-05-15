# tests/test_main_page.py
import pytest
from app import app
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import threading
import socket
from contextlib import closing


# Shared fixture for the Flask app
@pytest.fixture(scope="class")
def live_server():
    """Start Flask app once for all navigation tests"""
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.listen(1)
        port = s.getsockname()[1]
    
    
    server_thread = threading.Thread(
        target=lambda: app.run(host='127.0.0.1', port=port, debug=False, use_reloader=False)
    )
    server_thread.daemon = True
    server_thread.start()
    time.sleep(2)
    
    yield f"http://127.0.0.1:{port}"


class TestMainPageRoutes:
    """Test the main page route and basic functionality"""
    
    @pytest.fixture
    def client(self):
        """Create a test client for the Flask app"""
        app.config['TESTING'] = True
        with app.test_client() as client:
            yield client
    
    def test_main_route_loads(self, client):
        """Test that the main route '/' loads successfully"""
        response = client.get('/')
        assert response.status_code == 200
    
    def test_main_route_uses_correct_template(self, client):
        """Test that main route uses the main.html template"""
        response = client.get('/')
        assert b'Welcome to MAPF RoboSim' in response.data
        assert b'Multi-Agent Pathfinding Simulation System' in response.data
    
    def test_main_route_has_correct_title(self, client):
        """Test that the page has the correct title"""
        response = client.get('/')
        assert b'<title>MAPF Simulator</title>' in response.data
    
    def test_main_route_has_navigation_header(self, client):
        """Test that the navigation header exists"""
        response = client.get('/')
        assert b'RoboSim' in response.data
        assert b'nav' in response.data
    
    def test_main_route_has_buttons(self, client):
        """Test that both main buttons exist"""
        response = client.get('/')
        assert b'Continue' in response.data
        assert b'How it Works!' in response.data


class TestMainPageNavigation:
    """Test button clicks and navigation on the main page"""
    
    @pytest.fixture(autouse=True)
    def setup_selenium(self, live_server):
        """Setup Selenium webdriver"""
        self.driver = webdriver.Chrome()
        self.driver.implicitly_wait(5)
        self.base_url = live_server
        
        yield
        
        self.driver.quit()
    
    def test_continue_button_works(self):
        """Test that the Continue button navigates to home page"""
        # Go to main page
        self.driver.get(self.base_url)
        
        # Find and click the Continue button
        continue_button = WebDriverWait(self.driver, 10).until(
            EC.element_to_be_clickable((By.LINK_TEXT, "Continue"))
        )
        continue_button.click()
        
        # Wait for navigation and check we're on home page
        WebDriverWait(self.driver, 10).until(
            EC.url_contains("/home")
        )
        
        # Verify we're on the home page
        assert "/home" in self.driver.current_url
    
    def test_how_it_works_button_opens_info_page(self):
        """Test that 'How it Works!' button opens info page in new tab"""
        # Go to main page
        self.driver.get(self.base_url)
        
        # Get current number of windows
        initial_windows = len(self.driver.window_handles)
        
        # Find and click the "How it Works!" button
        how_it_works_button = WebDriverWait(self.driver, 10).until(
            EC.element_to_be_clickable((By.LINK_TEXT, "How it Works!"))
        )
        how_it_works_button.click()
        
        # Wait for new window to open
        WebDriverWait(self.driver, 10).until(
            lambda driver: len(driver.window_handles) > initial_windows
        )
        
        # Switch to new window and verify it's the info page
        new_window = self.driver.window_handles[-1]
        self.driver.switch_to.window(new_window)
        
        assert "/info" in self.driver.current_url
    
    def test_navigation_links_work(self):
        """Test all navigation menu links work"""
        # Go to main page
        self.driver.get(self.base_url)
        
        # Test navigation links
        nav_tests = [
            ("Home", "/home"),
            ("Instructions", "/info"),
            ("About", "/about"),
            ("Main", "/")  # This should go back to main
        ]
        
        for link_text, expected_url_part in nav_tests:
            # Find and click navigation link
            nav_link = WebDriverWait(self.driver, 10).until(
                EC.element_to_be_clickable((By.LINK_TEXT, link_text))
            )
            nav_link.click()
            
            # Wait for navigation
            time.sleep(1)
            
            # Verify we're on the expected page
            assert expected_url_part in self.driver.current_url, f"Failed navigation to {link_text}"
            
            # Go back to main page for next test
            if expected_url_part != "/":
                self.driver.get(self.base_url)
    
    def test_page_title_in_browser(self):
        """Test that the browser tab shows correct title"""
        self.driver.get(self.base_url)
        assert self.driver.title == "MAPF Simulator"
    
    def test_logo_image_exists(self):
        """Test that the robot logo exists in the header"""
        self.driver.get(self.base_url)
        
        # Find the logo image
        logo = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "header img"))
        )
        
        # Verify it's the correct image
        assert "icons8-robot-2-30.png" in logo.get_attribute("src")
    
    def test_hero_section_content(self):
        """Test that the hero section contains expected content"""
        self.driver.get(self.base_url)
        
        # Check main heading
        heading = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.TAG_NAME, "h1"))
        )
        assert "Welcome to MAPF RoboSim" in heading.text
        
        # Check paragraphs
        paragraphs = self.driver.find_elements(By.CSS_SELECTOR, ".hero-section p")
        assert len(paragraphs) >= 3
        assert "Multi-Agent Pathfinding Simulation System" in paragraphs[0].text
    
    def test_buttons_are_styled_correctly(self):
        """Test that buttons have the correct CSS classes"""
        self.driver.get(self.base_url)
        
        # Check Continue button styling - look for the button element inside the link
        continue_link = self.driver.find_element(By.LINK_TEXT, "Continue")
        continue_button = continue_link.find_element(By.TAG_NAME, "button")
        button_classes = continue_button.get_attribute("class")
        assert "btn" in button_classes
        assert "btn-outline-light" in button_classes
        assert "button-color" in button_classes
        assert "mt-3" in button_classes
        
        # Check How it Works button styling - also wrapped in a link
        how_it_works_link = self.driver.find_element(By.LINK_TEXT, "How it Works!")
        how_it_works_button = how_it_works_link.find_element(By.TAG_NAME, "button")
        button_classes = how_it_works_button.get_attribute("class")
        assert "btn" in button_classes
        assert "btn-outline-dark" in button_classes
        assert "mt-3" in button_classes